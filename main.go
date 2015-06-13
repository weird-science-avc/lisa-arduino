package main

import (
	"fmt"
	"log"
	"math"
	"time"
)

const (
	// The amount of units to be considered "at" a location
	ARRIVE_DELTA = 0.05
	// The amount of units at which to start slowing down when approaching a location
	APPROACH_DELTA = 3.0
	// The maximum speed to go when we're not close to our target
	// TODO: Should probably pick some units here
	MAX_SPEED = 10.0
)

type Vector struct {
	magnitude float64
	direction float64
}

func (v Vector) String() string {
	return fmt.Sprintf("(mag:%0.3f,dir:%0.3f)", v.magnitude, v.direction*180.0/math.Pi)
}

type Location struct {
	x float64
	y float64
}

func (l Location) String() string {
	return fmt.Sprintf("(%0.3f,%0.3f)", l.x, l.y)
}

type Position struct {
	loc         Location
	orientation float64
}

func (p Position) String() string {
	return fmt.Sprintf("%s:%0.3f", p.loc, p.orientation*180.0/math.Pi)
}

//func main() {
//	startPosiition = foo;
//	NavigateTo(0, 9.5)
//	//NavigateTo(-0.5, 10)
//	NavigateRightTurn()
//	position = (-0.5, 10)
//	NavigateTo(-9.5, 10)
//	NavigateTo(-10, 9.5)
//	NavigateTo(-10, 0.5)
//	NavigateTo(-9.5, 0)
//	NavigateTo(0, 0)
//}

func main() {
	// TODO: Take in as parameter, or an array we iterate through
	targetLocation := Location{0.0, 10.0}
	//targetLocation := Location{5.0, 5.0}

	// We always start at 0, 0 and along Y axis (pi/2)
	currentPosition := Position{
		loc:         Location{0.0, 0.0},
		orientation: math.Pi / 2.0,
	}

	// Create speed and steering services to talk to
	speedService, speedChan := NewSpeedService()
	steeringService, steeringChan := NewSteeringService()

	// Create a position channel using the speed and steering channel
	positionChan := makePositionChan(speedChan, steeringChan)

	// Do until we reach our target
	log.Println("Traversing from %s to %s", currentPosition, targetLocation)
	for !arrivedLocation(currentPosition.loc, targetLocation) {
		// Get the direct vector to our target
		targetVector := getDirectVector(currentPosition.loc, targetLocation)

		// TODO: Once we have sensors for items, we want to ask something if our
		// direct vector hits something, and if so to pick the tangental vector
		// that either has the smallest angle to the direct, or the smallest angle
		// to the origin to target vector
		// TODO: If we're going to hit something, i.e. if we can achieve our
		// avoidVector, then we might want to do something like stop and go
		// backwards a little bit
		//if avoidVector, ok := detectCollision(currentLocation, directVector, currentSurroundings); !ok {
		//	targetVector = avoidVector
		//}

		// Calculate target steering
		// TODO: left is negative, is that right?
		// NOTE: Allow excess of what the car can do, the steering control will
		// translate and limit as appropriate and we'll just keep trying to reach it
		targetSteering := int((targetVector.direction * 180.0 / math.Pi) - (currentPosition.orientation * 180.0 / math.Pi))

		// Calculate target speed with boundaries
		desiredSpeed := targetVector.magnitude / APPROACH_DELTA * float64(MAX_SPEED)
		targetSpeed := int(math.Min(math.Max(desiredSpeed, 1.0), MAX_SPEED))

		// And finally send speed and steering updates based on target vector
		// TODO: Should we just send floats and let the service choose what it wants
		speedService.Send(targetSpeed)
		steeringService.Send(targetSteering)
		// And report where we're at and what we're doing
		log.Printf("%s -- speed: %03d -- steering: %03d\n", currentPosition, targetSpeed, targetSteering)

		// Wait for new position information
		currentPosition = <-positionChan
	}
	// Fully stop since we're at our final location
	speedService.Send(0)
}

// Calculates the direct vector between two locations
func getDirectVector(current, target Location) (v Vector) {
	deltaY := target.y - current.y
	deltaX := target.x - current.x
	v.magnitude = math.Sqrt(math.Pow(deltaX, 2) + math.Pow(deltaY, 2))
	v.direction = math.Atan2(deltaY, deltaX)
	return
}

// Decides if we've arrived at a location
func arrivedLocation(current, target Location) bool {
	v := getDirectVector(current, target)
	return v.magnitude < ARRIVE_DELTA
}

type speedService struct {
	notificationChan chan int
}

func NewSpeedService() (*speedService, <-chan int) {
	c := make(chan int)
	return &speedService{c}, c
}

func (s *speedService) Send(value int) {
	// TODO: Should actually set here
	// Send to our notification channel
	s.notificationChan <- value
}

type steeringService struct {
	notificationChan chan int
}

func NewSteeringService() (*steeringService, <-chan int) {
	c := make(chan int)
	return &steeringService{c}, c
}

func (s *steeringService) Send(value int) {
	newValue := int(math.Min(math.Max(float64(value), -30.0), 30.0))
	// TODO: Should actually set here
	// Send to our notification channel
	s.notificationChan <- newValue
}

func makePositionChan(speedChan, steeringChan <-chan int) <-chan Position {
	// we start at (0, 0) pointing y-axis (math.Pi/2.0) with speed 0, steering 0
	x := 0.0
	y := 0.0
	orientation := math.Pi / 2.0
	speed := 0
	steering := 0

	// Kick off go rountine to wait for speed/steering/heartbeat, update position, send out notice
	c := make(chan Position, 1)
	go func() {
		timestamp := time.Now()
		newSpeed := speed
		newSteering := steering
		tick := time.Tick(100 * time.Millisecond)
		for {
			// Wait for speed/steering/tick updating appropriate data
			select {
			case <-tick:
			case newSpeed = <-speedChan:
				if newSpeed == speed {
					continue
				}
			case newSteering = <-steeringChan:
				if newSteering == steering {
					continue
				}
			}
			newTimestamp := time.Now()

			// Using speed, steering and timeElapsed update x, y, and orientation
			// TODO: Pick units so we know how to use with timeElapsed
			timeElapsed := newTimestamp.Sub(timestamp)
			// First get radius, velocity and distance traveled
			r := radiusFromSteering(steering)
			v := velocityFromSpeed(speed)
			d := v * timeElapsed.Seconds()
			// If radius is 0 then use straight direction math, otherwise banked math
			if r == 0.0 {
				// Straight motion
				x = x + d*math.Cos(orientation)
				y = y + d*math.Sin(orientation)
			} else {
				// Banked motion
				// FIXME: Need to adjust for orientation too
				x = r * (math.Cos((d/r)+math.Atan(y/(x+r))) - 1)
				y = r * math.Sin((d/r)+math.Atan(y/(x+r)))

				// orientation changes by straight angle from
				orientation -= d / r
			}

			// Send the data to the channel if available, otherwise just skip until next chance
			select {
			case c <- Position{loc: Location{x, y}, orientation: orientation}:
			default:
			}

			// Set data for next loop
			speed = newSpeed
			steering = newSteering
			timestamp = newTimestamp
		}
	}()
	return c
}

func radiusFromSteering(steering int) float64 {
	if steering == 0 {
		return 0.0
	} else {
		return 10.0 + float64(30-steering)/30.0
	}
}

func velocityFromSpeed(speed int) float64 {
	return float64(speed) / 10.0
}
