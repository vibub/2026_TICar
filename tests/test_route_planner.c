#include <stdint.h>
#include <stdio.h>

#include "route_planner.h"

#define CHECK(condition)                                                     \
    do {                                                                     \
        if (!(condition)) {                                                  \
            fprintf(stderr, "CHECK failed at line %d: %s\n", __LINE__,    \
                    #condition);                                             \
            return 0;                                                        \
        }                                                                    \
    } while (0)


static RoutePlanner_Observation observation(
    uint8_t digit,
    uint8_t side,
    uint8_t direction_mask,
    uint8_t confirmed)
{
    RoutePlanner_Observation result = {0};

    if (digit != 0U) {
        result.digit_valid = 1U;
        result.digit = digit;
        result.side = side;
        result.digit_flags = K230_DIGIT_FLAG_VALID |
                             K230_DIGIT_FLAG_TARGET_MATCH;
        if (confirmed != 0U) {
            result.digit_flags |= K230_DIGIT_FLAG_CONSENSUS;
        }
    }
    result.direction_mask = direction_mask;
    return result;
}

static int test_random_digits_follow_observed_side(void)
{
    uint8_t digit;

    for (digit = 3U; digit <= 8U; digit++) {
        RoutePlanner planner;
        RoutePlanner_Observation left = observation(
            digit, K230_DIGIT_SIDE_LEFT,
            K230_LINE_DIRECTION_LEFT | K230_LINE_DIRECTION_FRONT, 1U);
        RoutePlanner_Observation right = observation(
            digit, K230_DIGIT_SIDE_RIGHT,
            K230_LINE_DIRECTION_RIGHT | K230_LINE_DIRECTION_FRONT, 1U);

        RoutePlanner_Init(&planner);
        CHECK(RoutePlanner_SetTarget(&planner, digit) == 1U);
        CHECK(RoutePlanner_SetRegion(&planner, K230_ROUTE_REGION_MIDDLE) == 1U);
        CHECK(RoutePlanner_Decide(&planner, 1U, &left) == ROUTE_DECISION_LEFT);

        RoutePlanner_ClearJunction(&planner, 1U);
        CHECK(RoutePlanner_Decide(&planner, 2U, &right) == ROUTE_DECISION_RIGHT);
    }
    return 1;
}

static int test_non_target_at_middle_continues_front(void)
{
    RoutePlanner planner;
    RoutePlanner_Observation other = observation(
        4U, K230_DIGIT_SIDE_LEFT,
        K230_LINE_DIRECTION_LEFT | K230_LINE_DIRECTION_FRONT, 1U);

    RoutePlanner_Init(&planner);
    CHECK(RoutePlanner_SetTarget(&planner, 6U) == 1U);
    CHECK(RoutePlanner_SetRegion(&planner, K230_ROUTE_REGION_MIDDLE) == 1U);
    CHECK(RoutePlanner_Decide(&planner, 3U, &other) == ROUTE_DECISION_FRONT);
    return 1;
}

static int test_direction_mask_blocks_unsafe_turn(void)
{
    RoutePlanner planner;
    RoutePlanner_Observation target = observation(
        6U, K230_DIGIT_SIDE_LEFT, K230_LINE_DIRECTION_FRONT, 1U);

    RoutePlanner_Init(&planner);
    CHECK(RoutePlanner_SetTarget(&planner, 6U) == 1U);
    CHECK(RoutePlanner_SetRegion(&planner, K230_ROUTE_REGION_FAR) == 1U);
    CHECK(RoutePlanner_Decide(&planner, 4U, &target) == ROUTE_DECISION_HOLD);
    return 1;
}

static int test_near_room_fixed_direction_still_checks_line(void)
{
    RoutePlanner planner;
    RoutePlanner_Observation junction = observation(
        0U, K230_DIGIT_SIDE_CENTER,
        K230_LINE_DIRECTION_LEFT | K230_LINE_DIRECTION_RIGHT, 0U);

    RoutePlanner_Init(&planner);
    CHECK(RoutePlanner_SetTarget(&planner, 1U) == 1U);
    CHECK(RoutePlanner_SetRegion(&planner, K230_ROUTE_REGION_NEAR) == 1U);
    CHECK(RoutePlanner_Decide(&planner, 5U, &junction) == ROUTE_DECISION_LEFT);

    RoutePlanner_Init(&planner);
    CHECK(RoutePlanner_SetTarget(&planner, 2U) == 1U);
    CHECK(RoutePlanner_SetRegion(&planner, K230_ROUTE_REGION_NEAR) == 1U);
    CHECK(RoutePlanner_Decide(&planner, 5U, &junction) == ROUTE_DECISION_RIGHT);
    return 1;
}

static int test_same_junction_is_latched(void)
{
    RoutePlanner planner;
    RoutePlanner_Observation left = observation(
        7U, K230_DIGIT_SIDE_LEFT,
        K230_LINE_DIRECTION_LEFT | K230_LINE_DIRECTION_FRONT, 1U);
    RoutePlanner_Observation right = observation(
        7U, K230_DIGIT_SIDE_RIGHT,
        K230_LINE_DIRECTION_RIGHT | K230_LINE_DIRECTION_FRONT, 1U);

    RoutePlanner_Init(&planner);
    CHECK(RoutePlanner_SetTarget(&planner, 7U) == 1U);
    CHECK(RoutePlanner_SetRegion(&planner, K230_ROUTE_REGION_FAR) == 1U);
    CHECK(RoutePlanner_Decide(&planner, 6U, &left) == ROUTE_DECISION_LEFT);
    CHECK(RoutePlanner_Decide(&planner, 6U, &right) == ROUTE_DECISION_LEFT);
    return 1;
}

static int test_path_and_return_are_reversed(void)
{
    RoutePlanner planner;
    RoutePlanner_Observation left = observation(
        5U, K230_DIGIT_SIDE_LEFT,
        K230_LINE_DIRECTION_LEFT | K230_LINE_DIRECTION_FRONT, 1U);
    RoutePlanner_Observation front = observation(
        0U, K230_DIGIT_SIDE_CENTER, K230_LINE_DIRECTION_FRONT, 0U);
    RoutePlanner_Step step;

    RoutePlanner_Init(&planner);
    CHECK(RoutePlanner_SetTarget(&planner, 5U) == 1U);
    CHECK(RoutePlanner_SetRegion(&planner, K230_ROUTE_REGION_MIDDLE) == 1U);
    CHECK(RoutePlanner_Decide(&planner, 1U, &front) == ROUTE_DECISION_FRONT);
    CHECK(RoutePlanner_CommitDecision(
              &planner, 1U, K230_ROUTE_REGION_FAR) == 1U);
    RoutePlanner_ClearJunction(&planner, 1U);

    CHECK(RoutePlanner_Decide(&planner, 2U, &left) == ROUTE_DECISION_LEFT);
    CHECK(RoutePlanner_CommitDecision(
              &planner, 2U, K230_ROUTE_REGION_FAR_LEFT) == 1U);

    RoutePlanner_BeginReturn(&planner);
    CHECK(RoutePlanner_TakeReturnDecision(&planner, &step) == 1U);
    CHECK(step.decision == ROUTE_DECISION_RIGHT);
    CHECK(step.from_region == K230_ROUTE_REGION_FAR_LEFT);
    CHECK(step.to_region == K230_ROUTE_REGION_FAR);
    CHECK(RoutePlanner_TakeReturnDecision(&planner, &step) == 1U);
    CHECK(step.decision == ROUTE_DECISION_FRONT);
    CHECK(RoutePlanner_TakeReturnDecision(&planner, &step) == 0U);
    return 1;
}

int main(void)
{
    if (!test_random_digits_follow_observed_side() ||
        !test_non_target_at_middle_continues_front() ||
        !test_direction_mask_blocks_unsafe_turn() ||
        !test_near_room_fixed_direction_still_checks_line() ||
        !test_same_junction_is_latched() ||
        !test_path_and_return_are_reversed()) {
        return 1;
    }

    puts("route_planner tests passed");
    return 0;
}
