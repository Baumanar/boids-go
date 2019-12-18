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

	Acceleration_Limit = 5
	Speed_Limit = 10

	Separation_Distance = 3
	Separation_Factor = 1
	Alignment_Distance = 100
	Alignment_Factor = 3
	Cohesion_Distance = 100
	Cohesion_Factor = 3
)


var (
	running = true

	window_width = 1024
	window_height = 768
	World_Size_Width = 1400.0
	World_Size_Height = 1200.0
	population_size = 50

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
	fmt.Println(square_speed)
	if square_speed > ( Speed_Limit ) {
		b.velocity = b.velocity.Scaled( Speed_Limit / math.Sqrt( square_speed ) )
	}
}


func (b *Boid) updateBoid (otherBoids []*Boid, outBoid chan *Boid){
	updatedBoid := new(Boid)
	*updatedBoid = *b
	var acceleration pixel.Vec
	fmt.Println("last position: ", updatedBoid.position)


	boidsNear := b.boids_near(otherBoids, 200)

	var vec_align pixel.Vec
	count_align := 0
	var vec_cohesion pixel.Vec
	count_cohesion := 0
	var vec_separation pixel.Vec
	count_separation := 0

	//fmt.Println("number of near boids: ", len(boidsNear))
	for _, bNear := range boidsNear{


			diff_vector := b.diff_vector(bNear)
			//fmt.Println("diff vector: ", diff_vector)
			distance := b.distance_to(bNear)
		//fmt.Println("distance: ", distance)
			if distance < Alignment_Distance {

				vec_align = vec_align.Add( bNear.velocity)
				count_align++
			}

			if distance < Cohesion_Distance {
				vec_cohesion = vec_cohesion.Add( bNear.position)
				count_cohesion++
			}

			if distance < Separation_Distance {
				vec_separation = vec_separation.Add(diff_vector).Scaled(distance)
				count_separation++
			}

//			square_acceleration_magnitude := norm(acceleration)
//
//			if ( square_acceleration_magnitude > (  Acceleration_Limit ) ) {
//				acceleration = acceleration.Scaled( Acceleration_Limit / math.Sqrt( square_acceleration_magnitude ) )
//			}

	}
	vec_align = vec_align.Scaled(1.0/count_align)
	vec_cohesion = vec_align.Scaled(1.0/count_cohesion)
	vec_separation = vec_align.Scaled(1.0/count_separation)
	acceleration = acceleration.Add(vec_align).Add(vec_cohesion).Add(vec_separation)
	//fmt.Println("acc: ", acceleration)
	//fmt.Println("speed: ", updatedBoid.velocity)

	updatedBoid.velocity = updatedBoid.velocity.Add( acceleration )
	updatedBoid.limitVelo()
	updatedBoid.position = updatedBoid.position.Add( updatedBoid.velocity)
	fmt.Println("\nnew position: ", updatedBoid.position, updatedBoid.velocity)

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
	new_boid.position.X = rand.Float64() * World_Size_Width - 400;
	new_boid.position.Y = rand.Float64() * World_Size_Height - 400;
	new_boid.velocity.X = rand.Float64() * 2 ;
	new_boid.velocity.Y = rand.Float64() * 2 ;
	new_boid.last_updated = time.Now()
	new_boid.color = pixel.RGB(1, 0, 0)

	return new_boid
}

func loop_state(last_state []*Boid,  state_channel chan []*Boid, boid_updates chan *Boid) {
	//fmt.Println("started updating")
		new_state := make( []*Boid, population_size )
		fmt.Println("in loop state: ")
		fmt.Println("\t before update: ", last_state[0].position)
		for i := 0; i < len( last_state ); i++ {
			//fmt.Println(last_state[i].position)
			//if last_state != nil && len( last_state ) > i {
				//fmt.Println( last_state[ i ].velocity.X, last_state[ i ].velocity.Y)
				go last_state[ i ].updateBoid(last_state, boid_updates )
			//}
		}
		for i := 0; i < len( last_state ); i++ {
			x, ok := <-boid_updates
			if ok{
				new_state[i] = x
			}
		}
		state_channel <- new_state

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
	fmt.Println("at start:", last_state[0].position)
	state_channel := make( chan []*Boid )
	boid_updates := make( chan *Boid, 1 )
	for !win.Closed() {
		win.Clear(colornames.White)
		go loop_state(last_state, state_channel, boid_updates)
		x, ok := <- state_channel
		if ok {
			last_state = x
			fmt.Println("got value", x[0].position)
			draw_state( x, win )

		}
		time.Sleep(50 * time.Millisecond)
		win.Update()
	}
}


func main() {
	pixelgl.Run(run)
}

func draw_state( state []*Boid, win *pixelgl.Window) {

	for _, boid := range state {
		boid.drawBoid(win)
	}

}


func (b *Boid) drawBoid(win *pixelgl.Window) {
	mat := pixel.IM
	imd := imdraw.New(nil)
	imd.Color = b.color
	imd.Push(b.position)
	fmt.Println("in draw pos: ", b.position)
	vec := b.position.Sub(pixel.V(-20, 7))
	imd.Push(vec)
	vec = b.position.Add(pixel.V(20, 7))
	//imd.Push(pixel.V(600, 600))
	imd.Push(vec)
	mat = mat.Rotated(b.position,  math.Atan2( b.velocity.X, b.velocity.Y ))
	imd.Color = b.color
	imd.SetMatrix(mat)
	imd.Polygon(0)
	imd.Draw(win)

}