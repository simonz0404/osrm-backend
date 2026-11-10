#include "engine/routing_algorithms/many_to_many_early_stop.hpp"

#include <boost/test/unit_test.hpp>

#include <array>
#include <cstdint>
#include <limits>

BOOST_AUTO_TEST_SUITE(many_to_many_early_stop)

BOOST_AUTO_TEST_CASE(requires_finite_result_and_strict_weight_improvement_bound)
{
    using osrm::engine::routing_algorithms::ch::shouldStopSearch;
    BOOST_CHECK(!shouldStopSearch(EdgeWeight{10}, EdgeWeight{2}, INVALID_EDGE_WEIGHT, 0));
    BOOST_CHECK(!shouldStopSearch(EdgeWeight{10}, INVALID_EDGE_WEIGHT, EdgeWeight{2}, 0));
    BOOST_CHECK(!shouldStopSearch(EdgeWeight{10}, EdgeWeight{-2}, EdgeWeight{8}, 0));
    BOOST_CHECK(shouldStopSearch(EdgeWeight{10}, EdgeWeight{-1}, EdgeWeight{8}, 0));
}

BOOST_AUTO_TEST_CASE(excludes_open_area_loops_and_widens_arithmetic)
{
    using osrm::engine::routing_algorithms::ch::shouldStopSearch;
    BOOST_CHECK(!shouldStopSearch(EdgeWeight{10}, EdgeWeight{2}, EdgeWeight{8}, 13));
    BOOST_CHECK(shouldStopSearch(EdgeWeight{10}, EdgeWeight{2}, EdgeWeight{8}, 12));
    const auto maximum = std::numeric_limits<EdgeWeight::value_type>::max();
    BOOST_CHECK(
        shouldStopSearch(EdgeWeight{maximum - 10}, EdgeWeight{100}, EdgeWeight{maximum - 1}, 0));
    BOOST_CHECK(!shouldStopSearch(EdgeWeight{maximum - 10},
                                  EdgeWeight{100},
                                  EdgeWeight{maximum - 1},
                                  std::int64_t{maximum} * 2));
}

BOOST_AUTO_TEST_CASE(finishes_targets_independently_and_does_not_finish_unreachable_cells)
{
    const std::array minima{EdgeWeight{-2}, EdgeWeight{3}, EdgeWeight{0}};
    const std::array<std::int64_t, 3> approaches{0, 20, 0};
    osrm::engine::routing_algorithms::ch::ManyToManyEarlyStop state{minima, approaches, 0};
    state.updateTarget(0, EdgeWeight{8});
    state.updateTarget(1, EdgeWeight{5});
    BOOST_CHECK(!state.advance(EdgeWeight{10}));
    BOOST_CHECK(!state.isFinished(0));
    BOOST_CHECK(!state.isFinished(1));
    BOOST_CHECK(!state.advance(EdgeWeight{11}));
    BOOST_CHECK(state.isFinished(0));
    BOOST_CHECK(!state.isFinished(1));
    BOOST_CHECK(!state.advance(EdgeWeight{17}));
    BOOST_CHECK(state.isFinished(1));
    BOOST_CHECK(!state.isFinished(2));
    state.updateTarget(2, EdgeWeight{18});
    BOOST_CHECK(!state.advance(EdgeWeight{18}));
    BOOST_CHECK(state.advance(EdgeWeight{19}));
}

BOOST_AUTO_TEST_CASE(rechecks_when_a_better_result_lowers_a_threshold)
{
    const std::array minima{EdgeWeight{2}, INVALID_EDGE_WEIGHT};
    const std::array<std::int64_t, 2> approaches{0, 0};
    osrm::engine::routing_algorithms::ch::ManyToManyEarlyStop state{minima, approaches, 0};
    BOOST_CHECK(state.isFinished(1));
    state.updateTarget(0, EdgeWeight{100});
    BOOST_CHECK(!state.advance(EdgeWeight{10}));
    state.updateTarget(0, EdgeWeight{10});
    BOOST_CHECK(state.advance(EdgeWeight{10}));
}

BOOST_AUTO_TEST_CASE(threshold_matches_direct_single_target_predicate_exhaustively)
{
    using osrm::engine::routing_algorithms::ch::shouldStopSearch;
    unsigned unsafe = 0;
    for (int next = -20; next <= 20; ++next)
        for (int bucket = -20; bucket <= 20; ++bucket)
            for (int best = 0; best <= 20; ++best)
                for (int approach = 0; approach <= 30; ++approach)
                {
                    const bool expected = next + bucket > best && next + bucket >= approach;
                    unsafe += shouldStopSearch(EdgeWeight{next},
                                               EdgeWeight{bucket},
                                               EdgeWeight{best},
                                               approach) != expected;
                }
    BOOST_CHECK_EQUAL(unsafe, 0);
}

BOOST_AUTO_TEST_SUITE_END()
