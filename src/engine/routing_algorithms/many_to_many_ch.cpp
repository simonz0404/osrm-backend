#include "engine/routing_algorithms/many_to_many.hpp"
#include "engine/routing_algorithms/many_to_many_early_stop.hpp"
#include "engine/routing_algorithms/routing_base_ch.hpp"

#include <boost/assert.hpp>
#include <ranges>

#include <vector>

namespace osrm::engine::routing_algorithms
{

namespace ch
{

// The test is on weight minus approach, not on weight. A source and a target on one
// edge-based node with the target the earlier of the two need a loop to get from one to
// the other, and the graph part of the weight going negative is what says so. The walk
// into an open area is charged at both ends and is not travel, so it lifts the sum back
// above zero and hides the case. The cell then comes out short by the stretch between
// them. weight is the whole weight, walk included, which is what the cell reports;
// approach is how much of it is the walk.
inline bool addLoopWeight(const DataFacade<ch::Algorithm> &facade,
                          const NodeID node,
                          EdgeWeight &weight,
                          const EdgeWeight approach,
                          EdgeDuration &duration,
                          EdgeDistance &distance)
{ // Special case for CH when contractor creates a loop edge node->node
    BOOST_ASSERT(weight - approach < EdgeWeight{0});

    const auto loop_weight = ch::getLoopMetric<EdgeWeight>(facade, node);
    if (std::get<0>(loop_weight) != INVALID_EDGE_WEIGHT)
    {
        const auto new_weight_with_loop = weight + std::get<0>(loop_weight);
        if (new_weight_with_loop - approach >= EdgeWeight{0})
        {
            weight = new_weight_with_loop;
            auto result = ch::getLoopMetric<EdgeDuration>(facade, node);
            duration += std::get<0>(result);
            distance += std::get<1>(result);
            return true;
        }
    }

    // No loop found or adjusted weight is negative
    return false;
}

template <bool DIRECTION>
void relaxOutgoingEdges(
    const DataFacade<Algorithm> &facade,
    const typename SearchEngineData<Algorithm>::ManyToManyQueryHeap::HeapNode &heapNode,
    SearchEngineData<Algorithm>::ManyToManyQueryHeap &query_heap,
    const PhantomNodeCandidates &)
{
    if (stallAtNode<DIRECTION>(facade, heapNode, query_heap))
    {
        return;
    }

    for (auto edge : facade.GetAdjacentEdgeRange(heapNode.node))
    {
        const auto &data = facade.GetEdgeData(edge);
        if (DIRECTION == FORWARD_DIRECTION ? data.forward : data.backward)
        {
            const NodeID to = facade.GetTarget(edge);
            const auto edge_weight = data.weight;

            const auto edge_duration = data.duration;
            const auto edge_distance = data.distance;

            BOOST_ASSERT_MSG(edge_weight > EdgeWeight{0}, "edge_weight invalid");
            const auto to_weight = heapNode.weight + edge_weight;
            const auto to_duration = heapNode.data.duration + to_alias<EdgeDuration>(edge_duration);
            const auto to_distance = heapNode.data.distance + edge_distance;

            const auto toHeapNode = query_heap.GetHeapNodeIfWasInserted(to);
            // New Node discovered -> Add to Heap + Node Info Storage
            if (!toHeapNode)
            {
                query_heap.Insert(
                    to,
                    to_weight,
                    {heapNode.node, to_duration, to_distance, heapNode.data.approach});
            }
            // Found a shorter Path -> Update weight and set new parent
            else if (std::tie(to_weight, to_duration) <
                     std::tie(toHeapNode->weight, toHeapNode->data.duration))
            {
                toHeapNode->data = {
                    heapNode.node, to_duration, to_distance, heapNode.data.approach};
                toHeapNode->weight = to_weight;
                query_heap.DecreaseKey(*toHeapNode);
            }
        }
    }
}

void updateTableCell(const DataFacade<Algorithm> &facade,
                     const NodeID middle_node,
                     const EdgeWeight source_weight,
                     const ManyToManyHeapData &source_data,
                     const EdgeWeight target_weight,
                     const ManyToManyHeapData &target_data,
                     EdgeWeight &current_weight,
                     EdgeDuration &current_duration,
                     EdgeDistance &current_distance,
                     NodeID &middle_node_result)
{
    auto new_weight = source_weight + target_weight;
    auto new_duration = source_data.duration + target_data.duration;
    auto new_distance = source_data.distance + target_data.distance;
    const auto approach = source_data.approach + target_data.approach;
    if (new_weight - approach < EdgeWeight{0})
    {
        if (addLoopWeight(facade, middle_node, new_weight, approach, new_duration, new_distance))
        {
            current_weight = std::min(current_weight, new_weight);
            current_duration = std::min(current_duration, new_duration);
            current_distance = std::min(current_distance, new_distance);
            middle_node_result = middle_node;
        }
    }
    else if (std::tie(new_weight, new_duration) < std::tie(current_weight, current_duration))
    {
        current_weight = new_weight;
        current_duration = new_duration;
        current_distance = new_distance;
        middle_node_result = middle_node;
    }
}

template <bool MULTIPLE_TARGETS = false>
void forwardRoutingStep(const DataFacade<Algorithm> &facade,
                        const std::size_t row_index,
                        const std::size_t number_of_targets,
                        SearchEngineData<Algorithm>::ManyToManyQueryHeap &query_heap,
                        const std::vector<NodeBucket> &search_space_with_buckets,
                        std::vector<EdgeWeight> &weights_table,
                        std::vector<EdgeDuration> &durations_table,
                        std::vector<EdgeDistance> &distances_table,
                        std::vector<NodeID> &middle_nodes_table,
                        const PhantomNodeCandidates &candidates,
                        ManyToManyEarlyStop *early_stop = nullptr)
{
    // Take a copy of the extracted node because otherwise could be modified later if toHeapNode is
    // the same
    const auto heapNode = query_heap.DeleteMinGetHeapNode();

    // Check if each encountered node has an entry
    const auto &bucket_list = std::equal_range(search_space_with_buckets.begin(),
                                               search_space_with_buckets.end(),
                                               heapNode.node,
                                               NodeBucket::Compare());
    for (const auto &current_bucket : std::ranges::subrange(bucket_list.first, bucket_list.second))
    {
        // Get target id from bucket entry
        const auto column_index = current_bucket.column_index;
        if constexpr (MULTIPLE_TARGETS)
            if (early_stop->isFinished(column_index))
                continue;
        const auto target_weight = current_bucket.weight;
        const auto target_duration = current_bucket.duration;
        const auto target_distance = current_bucket.distance;

        auto &current_weight = weights_table[row_index * number_of_targets + column_index];

        EdgeDistance nulldistance = {0};

        auto &current_duration = durations_table[row_index * number_of_targets + column_index];
        auto &current_distance =
            distances_table.empty() ? nulldistance
                                    : distances_table[row_index * number_of_targets + column_index];

        [[maybe_unused]] const auto previous_weight = current_weight;
        updateTableCell(
            facade,
            heapNode.node,
            heapNode.weight,
            heapNode.data,
            target_weight,
            {current_bucket.parent_node, target_duration, target_distance, current_bucket.approach},
            current_weight,
            current_duration,
            current_distance,
            middle_nodes_table[row_index * number_of_targets + column_index]);
        if constexpr (MULTIPLE_TARGETS)
            if (previous_weight != current_weight)
                early_stop->updateTarget(column_index, current_weight);
    }

    relaxOutgoingEdges<FORWARD_DIRECTION>(facade, heapNode, query_heap, candidates);
}

void backwardRoutingStep(const DataFacade<Algorithm> &facade,
                         const unsigned column_index,
                         SearchEngineData<Algorithm>::ManyToManyQueryHeap &query_heap,
                         std::vector<NodeBucket> &search_space_with_buckets,
                         const PhantomNodeCandidates &candidates)
{
    // Take a copy (no ref &) of the extracted node because otherwise could be modified later if
    // toHeapNode is the same
    const auto heapNode = query_heap.DeleteMinGetHeapNode();

    // Store settled nodes in search space bucket
    search_space_with_buckets.emplace_back(heapNode.node,
                                           heapNode.data.parent,
                                           column_index,
                                           heapNode.weight,
                                           heapNode.data.duration,
                                           heapNode.data.distance,
                                           heapNode.data.approach);

    relaxOutgoingEdges<REVERSE_DIRECTION>(facade, heapNode, query_heap, candidates);
}

std::int64_t maximumApproach(const PhantomNodeCandidates &candidates)
{
    std::int64_t maximum = 0;
    for (const auto &candidate : candidates)
        maximum = std::max(maximum, from_alias<std::int64_t>(candidate.approach_weight));
    return maximum;
}

template <bool SOURCE> bool hasDuplicateSeeds(const PhantomNodeCandidates &candidates)
{
    std::vector<NodeID> seeds;
    for (const auto &candidate : candidates)
    {
        if (SOURCE ? candidate.IsValidForwardSource() : candidate.IsValidForwardTarget())
            seeds.push_back(candidate.forward_segment_id.id);
        if (SOURCE ? candidate.IsValidReverseSource() : candidate.IsValidReverseTarget())
            seeds.push_back(candidate.reverse_segment_id.id);
    }
    std::sort(seeds.begin(), seeds.end());
    return std::adjacent_find(seeds.begin(), seeds.end()) != seeds.end();
}

std::pair<std::vector<EdgeDuration>, std::vector<EdgeDistance>>
oneToManySearch(SearchEngineData<Algorithm> &engine_working_data,
                const DataFacade<Algorithm> &facade,
                const std::vector<PhantomNodeCandidates> &candidates_list,
                const std::size_t source_index,
                const std::vector<std::size_t> &target_indices,
                const bool calculate_distance)
{
    using HeapNode = SearchEngineData<Algorithm>::ManyToManyQueryHeap::HeapNode;
    struct SourceBucket
    {
        HeapNode heap_node;
        std::size_t settlement_order;
    };
    struct Meeting
    {
        std::size_t source_bucket;
        HeapNode target;
    };

    engine_working_data.InitializeOrClearManyToManyThreadLocalStorage(facade.GetNumberOfNodes());
    auto &query_heap = *engine_working_data.many_to_many_heap;
    const auto &source_candidates = candidates_list[source_index];
    insertSourceInHeap(query_heap, source_candidates);
    if (query_heap.Empty())
        return {std::vector<EdgeDuration>(target_indices.size(), MAXIMAL_EDGE_DURATION),
                std::vector<EdgeDistance>(calculate_distance ? target_indices.size() : 0,
                                          MAXIMAL_EDGE_DISTANCE)};

    const auto minimum_source_weight = query_heap.MinKey();
    const auto maximum_source_approach = maximumApproach(source_candidates);
    std::vector<SourceBucket> source_buckets;
    while (!query_heap.Empty())
    {
        const auto heap_node = query_heap.DeleteMinGetHeapNode();
        source_buckets.push_back({heap_node, source_buckets.size()});
        relaxOutgoingEdges<FORWARD_DIRECTION>(facade, heap_node, query_heap, source_candidates);
    }
    std::sort(source_buckets.begin(),
              source_buckets.end(),
              [](const auto &lhs, const auto &rhs)
              { return lhs.heap_node.node < rhs.heap_node.node; });

    std::vector<EdgeDuration> durations(target_indices.size(), MAXIMAL_EDGE_DURATION);
    std::vector<EdgeDistance> distances(calculate_distance ? target_indices.size() : 0,
                                        MAXIMAL_EDGE_DISTANCE);
    std::vector<Meeting> meetings;
    for (std::size_t column = 0; column < target_indices.size(); ++column)
    {
        engine_working_data.InitializeOrClearManyToManyThreadLocalStorage(
            facade.GetNumberOfNodes());
        const auto &target_candidates = candidates_list[target_indices[column]];
        insertTargetInHeap(query_heap, target_candidates);
        const auto maximum_approach = maximum_source_approach + maximumApproach(target_candidates);
        auto best_weight = INVALID_EDGE_WEIGHT;
        auto best_order = std::numeric_limits<std::size_t>::max();
        EdgeDistance unused_distance{0};
        auto &distance = calculate_distance ? distances[column] : unused_distance;
        bool has_loop = false;
        meetings.clear();
        while (!query_heap.Empty())
        {
            if (!has_loop &&
                shouldStopSearch(
                    query_heap.MinKey(), minimum_source_weight, best_weight, maximum_approach))
                break;

            const auto target = query_heap.DeleteMinGetHeapNode();
            const auto found = std::lower_bound(source_buckets.begin(),
                                                source_buckets.end(),
                                                target.node,
                                                [](const auto &bucket, const NodeID node)
                                                { return bucket.heap_node.node < node; });
            if (found != source_buckets.end() && found->heap_node.node == target.node)
            {
                const auto source_bucket = static_cast<std::size_t>(found - source_buckets.begin());
                meetings.push_back({source_bucket, target});
                const auto &source = found->heap_node;
                const auto weight = source.weight + target.weight;
                const auto duration = source.data.duration + target.data.duration;
                const auto approach = source.data.approach + target.data.approach;
                has_loop = has_loop || weight - approach < EdgeWeight{0};
                if (!has_loop && std::tie(weight, duration, found->settlement_order) <
                                     std::tie(best_weight, durations[column], best_order))
                {
                    best_weight = weight;
                    durations[column] = duration;
                    distance = source.data.distance + target.data.distance;
                    best_order = found->settlement_order;
                }
            }
            relaxOutgoingEdges<REVERSE_DIRECTION>(facade, target, query_heap, target_candidates);
        }
        if (has_loop)
        {
            // Loop minima are order-sensitive. Replay every meeting in upstream's forward order.
            std::sort(meetings.begin(),
                      meetings.end(),
                      [&](const auto &lhs, const auto &rhs)
                      {
                          return source_buckets[lhs.source_bucket].settlement_order <
                                 source_buckets[rhs.source_bucket].settlement_order;
                      });
            best_weight = INVALID_EDGE_WEIGHT;
            durations[column] = MAXIMAL_EDGE_DURATION;
            distance = MAXIMAL_EDGE_DISTANCE;
            NodeID middle_node = SPECIAL_NODEID;
            for (const auto &meeting : meetings)
            {
                const auto &source = source_buckets[meeting.source_bucket].heap_node;
                updateTableCell(facade,
                                source.node,
                                source.weight,
                                source.data,
                                meeting.target.weight,
                                meeting.target.data,
                                best_weight,
                                durations[column],
                                distance,
                                middle_node);
            }
        }
    }
    return {std::move(durations), std::move(distances)};
}

} // namespace ch

template <>
std::pair<std::vector<EdgeDuration>, std::vector<EdgeDistance>>
manyToManySearch(SearchEngineData<ch::Algorithm> &engine_working_data,
                 const DataFacade<ch::Algorithm> &facade,
                 const std::vector<PhantomNodeCandidates> &candidates_list,
                 const std::vector<std::size_t> &source_indices,
                 const std::vector<std::size_t> &target_indices,
                 const bool calculate_distance)
{
    const auto number_of_sources = source_indices.size();
    const auto number_of_targets = target_indices.size();
    const auto number_of_entries = number_of_sources * number_of_targets;

    if (number_of_sources == 1 && number_of_targets > 1 &&
        !ch::hasDuplicateSeeds<true>(candidates_list[source_indices.front()]) &&
        std::none_of(target_indices.begin(),
                     target_indices.end(),
                     [&](const auto index)
                     { return ch::hasDuplicateSeeds<false>(candidates_list[index]); }))
        return ch::oneToManySearch(engine_working_data,
                                   facade,
                                   candidates_list,
                                   source_indices.front(),
                                   target_indices,
                                   calculate_distance);

    std::vector<EdgeWeight> weights_table(number_of_entries, INVALID_EDGE_WEIGHT);
    std::vector<EdgeDuration> durations_table(number_of_entries, MAXIMAL_EDGE_DURATION);
    std::vector<EdgeDistance> distances_table(calculate_distance ? number_of_entries : 0,
                                              MAXIMAL_EDGE_DISTANCE);
    std::vector<NodeID> middle_nodes_table(number_of_entries, SPECIAL_NODEID);

    std::vector<NodeBucket> search_space_with_buckets;
    std::vector<EdgeWeight> minimum_target_weights(number_of_targets, INVALID_EDGE_WEIGHT);
    std::vector<std::int64_t> maximum_target_approaches(number_of_targets, 0);

    // Populate buckets with paths from all accessible nodes to destinations via backward searches
    for (std::uint32_t column_index = 0; column_index < target_indices.size(); ++column_index)
    {
        const auto index = target_indices[column_index];
        const auto &target_candidates = candidates_list[index];

        engine_working_data.InitializeOrClearManyToManyThreadLocalStorage(
            facade.GetNumberOfNodes());
        auto &query_heap = *(engine_working_data.many_to_many_heap);
        insertTargetInHeap(query_heap, target_candidates);
        if (!query_heap.Empty())
            minimum_target_weights[column_index] = query_heap.MinKey();
        maximum_target_approaches[column_index] = ch::maximumApproach(target_candidates);

        // Explore search space
        while (!query_heap.Empty())
        {
            backwardRoutingStep(
                facade, column_index, query_heap, search_space_with_buckets, target_candidates);
        }
    }

    // Order lookup buckets
    std::sort(search_space_with_buckets.begin(), search_space_with_buckets.end());

    // Find shortest paths from sources to all accessible nodes
    for (std::uint32_t row_index = 0; row_index < source_indices.size(); ++row_index)
    {
        const auto source_index = source_indices[row_index];
        const auto &source_candidates = candidates_list[source_index];

        // Clear heap and insert source nodes
        engine_working_data.InitializeOrClearManyToManyThreadLocalStorage(
            facade.GetNumberOfNodes());
        auto &query_heap = *(engine_working_data.many_to_many_heap);
        insertSourceInHeap(query_heap, source_candidates);

        const auto maximum_source_approach = ch::maximumApproach(source_candidates);
        if (number_of_targets == 1)
        {
            while (!query_heap.Empty())
            {
                if (ch::shouldStopSearch(query_heap.MinKey(),
                                         minimum_target_weights.front(),
                                         weights_table[row_index],
                                         maximum_source_approach +
                                             maximum_target_approaches.front()))
                    break;
                forwardRoutingStep(facade,
                                   row_index,
                                   number_of_targets,
                                   query_heap,
                                   search_space_with_buckets,
                                   weights_table,
                                   durations_table,
                                   distances_table,
                                   middle_nodes_table,
                                   source_candidates);
            }
        }
        else
        {
            ch::ManyToManyEarlyStop early_stop{
                minimum_target_weights, maximum_target_approaches, maximum_source_approach};
            while (!query_heap.Empty())
            {
                if (early_stop.advance(query_heap.MinKey()))
                    break;
                forwardRoutingStep<true>(facade,
                                         row_index,
                                         number_of_targets,
                                         query_heap,
                                         search_space_with_buckets,
                                         weights_table,
                                         durations_table,
                                         distances_table,
                                         middle_nodes_table,
                                         source_candidates,
                                         &early_stop);
            }
        }
    }

    return std::make_pair(std::move(durations_table), std::move(distances_table));
}

} // namespace osrm::engine::routing_algorithms
