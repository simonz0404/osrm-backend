#ifndef OSRM_ENGINE_ROUTING_ALGORITHMS_MANY_TO_MANY_EARLY_STOP_HPP
#define OSRM_ENGINE_ROUTING_ALGORITHMS_MANY_TO_MANY_EARLY_STOP_HPP

#include "util/typedefs.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

namespace osrm::engine::routing_algorithms::ch
{

inline std::int64_t searchTerminationThreshold(const EdgeWeight minimum_bucket_weight,
                                               const EdgeWeight best_weight,
                                               const std::int64_t maximum_approach)
{
    if (best_weight == INVALID_EDGE_WEIGHT || minimum_bucket_weight == INVALID_EDGE_WEIGHT)
        return std::numeric_limits<std::int64_t>::max();

    const auto minimum = from_alias<std::int64_t>(minimum_bucket_weight);
    // Keep equal-weight duration ties and exclude all remaining loop corrections.
    return std::max(from_alias<std::int64_t>(best_weight) - minimum + 1,
                    maximum_approach - minimum);
}

inline bool shouldStopSearch(const EdgeWeight next_weight,
                             const EdgeWeight minimum_bucket_weight,
                             const EdgeWeight best_weight,
                             const std::int64_t maximum_approach)
{
    return from_alias<std::int64_t>(next_weight) >=
           searchTerminationThreshold(minimum_bucket_weight, best_weight, maximum_approach);
}

class ManyToManyEarlyStop
{
  public:
    ManyToManyEarlyStop(const std::span<const EdgeWeight> minimum_target_weights,
                        const std::span<const std::int64_t> maximum_target_approaches,
                        const std::int64_t maximum_source_approach)
        : m_minimum_target_weights(minimum_target_weights),
          m_maximum_target_approaches(maximum_target_approaches),
          m_maximum_source_approach(maximum_source_approach),
          m_thresholds(minimum_target_weights.size(), m_infinity),
          m_finished(minimum_target_weights.size(), false),
          m_remaining(minimum_target_weights.size())
    {
        for (std::size_t column = 0; column < minimum_target_weights.size(); ++column)
            if (minimum_target_weights[column] == INVALID_EDGE_WEIGHT)
            {
                m_finished[column] = true;
                --m_remaining;
            }
    }

    void updateTarget(const std::size_t column, const EdgeWeight best_weight)
    {
        m_thresholds[column] = searchTerminationThreshold(m_minimum_target_weights[column],
                                                          best_weight,
                                                          m_maximum_source_approach +
                                                              m_maximum_target_approaches[column]);
        m_next_threshold = std::min(m_next_threshold, m_thresholds[column]);
    }

    bool advance(const EdgeWeight next_weight)
    {
        const auto next = from_alias<std::int64_t>(next_weight);
        if (next >= m_next_threshold)
        {
            m_next_threshold = m_infinity;
            for (std::size_t column = 0; column < m_thresholds.size(); ++column)
                if (!m_finished[column])
                {
                    if (next >= m_thresholds[column])
                    {
                        m_finished[column] = true;
                        --m_remaining;
                    }
                    else
                        m_next_threshold = std::min(m_next_threshold, m_thresholds[column]);
                }
        }
        return m_remaining == 0;
    }

    bool isFinished(const std::size_t column) const { return m_finished[column]; }

  private:
    static constexpr auto m_infinity = std::numeric_limits<std::int64_t>::max();
    const std::span<const EdgeWeight> m_minimum_target_weights;
    const std::span<const std::int64_t> m_maximum_target_approaches;
    const std::int64_t m_maximum_source_approach;
    std::vector<std::int64_t> m_thresholds;
    std::vector<bool> m_finished;
    std::size_t m_remaining;
    std::int64_t m_next_threshold = m_infinity;
};

} // namespace osrm::engine::routing_algorithms::ch

#endif
