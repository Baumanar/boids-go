package main

import (
	"fmt"
	"github.com/faiface/pixel"
	"github.com/faiface/pixel/imdraw"
	"github.com/faiface/pixel/pixelgl"
	"golang.org/x/image/colornames"
	"math"
	"math/rand"
	"time"
)


const (
	Acceleration_Limit = 10
	Speed_Limit = 20

	Separation_Distance = 5
	Separation_Factor = 30
	Alignment_Distance = 6
	Alignment_Factor = 20
	Cohesion_Distance = 12
	Cohesion_Factor = 10

)


var (
	running = true

	window_width = 1024
	window_height = 768
	World_Size_Width = 1200.0
	World_Size_Height = 800.0
	population_size = 1000

)


type Boid struct {
	position pixel.Vec
	velocity pixel.Vec
	last_updated time.Time
	color pixel.RGBA
}





func norm(vec pixel.Vec) float64 {
	return vec.X*vec.X + vec.Y*vec.Y
}


func (b *Boid) diff_vector(otherBoid *Boid) pixel.Vec{
	return b.position.Sub(otherBoid.position)
}

func (b *Boid) distance_to(otherBoid *Boid) float64 {
	vec := b.position
	other := otherBoid.position
	return math.Sqrt( ( other.X - vec.X ) * ( other.X - vec.X ) + ( other.Y - vec.Y ) * ( other.Y - vec.Y ) )
}

func (b *Boid) boids_near (otherBoids []*Boid, distance_min float64) []*Boid{
	boids_near := make([]*Boid, 0)
	for _, boid := range otherBoids{
		if b != boid{
			if(b.distance_to(boid)<distance_min){
				boids_near = append(boids_near, boid)
			}
		}
	}
	return boids_near
}

func (b *Boid) limitVelo(){
	square_speed := norm(b.velocity)
	if square_speed > ( Speed_Limit ) {
		b.velocity = b.velocity.Scaled( Speed_Limit / math.Sqrt( square_speed ) )
	}
}


func (b *Boid) updateBoid (otherBoids []*Boid, outBoid chan *Boid){
	updatedBoid := new(Boid)

	var acceleration pixel.Vec


	boidsNear := b.boids_near(otherBoids, 10)
	for _, bNear := range boidsNear{
			diff_vector := b.diff_vector(bNear)
			distance := b.distance_to(bNear)
			if distance < Separation_Distance {
				acceleration.Add( diff_vector.Scaled( -Separation_Factor ) )
			}

			if distance < Alignment_Distance {
				acceleration.Add( diff_vector.Scaled( Cohesion_Factor ) )
			}

			if distance < Alignment_Distance {
				acceleration.Add( bNear.velocity.Scaled( Alignment_Factor ) )
			}

			square_acceleration_magnitude := norm(acceleration)
			if ( square_acceleration_magnitude > ( Acceleration_Limit * Acceleration_Limit ) ) {
				acceleration = acceleration.Scaled( Acceleration_Limit / math.Sqrt( square_acceleration_magnitude ) )
			}


	}
	updatedBoid.velocity.Add( acceleration )
	updatedBoid.limitVelo()
	updatedBoid.position.Add( updatedBoid.velocity)

	for updatedBoid.position.X < 0 {
		updatedBoid.position.X += World_Size_Width
	}

	for updatedBoid.position.X > World_Size_Width {
		updatedBoid.position.X -= World_Size_Width
	}

	for updatedBoid.position.Y < 0 {
		updatedBoid.position.Y += World_Size_Height
	}

	for updatedBoid.position.Y > World_Size_Height {
		updatedBoid.position.Y -= World_Size_Height
	}

	outBoid <- updatedBoid
}


func create_boid () *Boid {
	new_boid := new( Boid )
	new_boid.position.X = rand.Float64() * World_Size_Width - 200;
	new_boid.position.Y = rand.Float64() * World_Size_Height - 200;
	new_boid.last_updated = time.Now()
	new_boid.color = pixel.RGB(1, 0, 0)

	return new_boid
}

func loop_state(last_state []*Boid,  state_channel chan []*Boid) {
	fmt.Println("started updating")
	boid_updates := make( chan *Boid, 1000 )
	for running {
		new_state := make( []*Boid, population_size )

		for i := 0; i < len( new_state ); i++ {
			if last_state != nil && len( last_state ) > i {
				//fmt.Println( last_state[ i ].velocity.X, last_state[ i ].velocity.Y)
				go last_state[ i ].updateBoid(last_state, boid_updates )
			}
		}
		for i := 0; i < len( new_state ); i++ {
			new_state[ i ] = <- boid_updates
		}
		state_channel <- new_state
		last_state = new_state
	}
}


func run() {
	cfg := pixelgl.WindowConfig{
		Title:  "Pixel Rocks!",
		Bounds: pixel.R(0, 0, 1200, 800),
		VSync:  true,
	}
	win, err := pixelgl.NewWindow(cfg)
	if err != nil {
		panic(err)
	}

	last_state := make([]*Boid, 0)
	for i:=0; i<population_size; i++ {
		last_state = append(last_state, create_boid())
	}

	for !win.Closed() {
		win.Clear(colornames.White)
		state_channel := make( chan []*Boid )
		go loop_state(last_state, state_channel)
		draw_state( <- state_channel, win )

		win.Update()
	}
}


func main() {
	pixelgl.Run(run)
}

func draw_state( state []*Boid, win *pixelgl.Window) {
	fmt.Println("started drawing")

	for _, boid := range state {
		boid.drawBoid(win)
	}

}


func (b *Boid) drawBoid(win *pixelgl.Window) {

	imd := imdraw.New(nil)
	imd.Color = b.color
	imd.Push(b.position)
	imd.Push(b.position.Add(pixel.V(20,20)))
	imd.Push(b.position.Sub(pixel.V(20,20)))
	imd.Polygon(0)
	imd.Draw(win)
}