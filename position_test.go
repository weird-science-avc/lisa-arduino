package main

import (
	"fmt"
	"math"
	"testing"
)

type testCase struct {
	p        Position // current position (x,y in m)
	d        float64  // distance traveled (m/s)
	r        float64  // turning radius (m)
	expected Position // expected end position
}

const floatTolerance = 0.01

func matchPosition(t *testing.T, actual, expected Position) {
	var message string

	// Find angle diff and make sure within tolerance
	rDiff := expected.r - actual.r
	if rDiff > floatTolerance {
		message = fmt.Sprintf("Failed orientation tolerance: %f", rDiff)
	}

	// Find distance and make sure its within tolerance
	d := math.Sqrt(math.Pow(expected.x-actual.x, 2) + math.Pow(expected.y-actual.y, 2))
	if d > floatTolerance {
		message = fmt.Sprintf("Failed distance tolerance: %f", d)
	}

	if message != "" {
		t.Errorf("%s:\n  expected: %v\n    actual: %v\n", message, expected, actual)
	}
}

//calculateNewPosition(position Position, distance, turnRadius float64)
func TestNoRadius(t *testing.T) {
	testCases := []testCase{
		// No turning
		{Position{0, 0, 0}, 1, math.NaN(),
			Position{1, 0, 0}},
		{Position{0, 0, math.Pi / 4}, 1, math.NaN(),
			Position{1 / math.Sqrt(2), 1 / math.Sqrt(2), math.Pi / 4}},
		{Position{0, 0, math.Pi / 2}, 1, math.NaN(),
			Position{0, 1, math.Pi / 2}},

		// Unit circles
		// Turn quarter circle left
		{Position{0, 0, 0}, math.Pi / 2, 1,
			Position{1, 1, math.Pi / 2}},

		// Turn half circle left
		{Position{0, 0, 0}, math.Pi, 1,
			Position{0, 2, math.Pi}},

		// Turn quarter circle left, starting facing towards y
		{Position{0, 0, math.Pi / 2}, math.Pi / 2, 1,
			Position{-1, 1, math.Pi}},

		// Turn quarter circle right
		{Position{0, 0, 0}, math.Pi / 2, -1,
			Position{1, -1, -math.Pi / 2}},

		// Turn quarter circle left at (1,1)
		{Position{1, 1, 0}, math.Pi / 2, 1,
			Position{2, 2, math.Pi / 2}},

		// Turn quarter circle left at (1,1), starting facing towards -y
		{Position{1, 1, -math.Pi / 2}, math.Pi / 2, 1,
			Position{2, 0, 0}},

		// Test Double unit circle
		// Turn quarter circle left
		{Position{0, 0, 0}, 2 * math.Pi / 2, 2,
			Position{2, 2, math.Pi / 2}},
	}
	for _, tc := range testCases {
		actual := calculateNewPosition(tc.p, tc.d, tc.r)
		matchPosition(t, actual, tc.expected)
	}
}
