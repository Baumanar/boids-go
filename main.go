package main

import (
	"github.com/faiface/pixel"
	"github.com/faiface/pixel/imdraw"
	"github.com/faiface/pixel/pixelgl"
	"golang.org/x/image/colornames"
	"math"
	"math/rand"
	"sync"
	"time"
)


const (
	Acceleration_Limit = 30
	Speed_Limit = 5
	Avoid_Distance = 200
	Separation_Distance = 8
	Alignment_Distance = 90
	Cohesion_Distance = 90
)


var (
	World_Size_Width = 1400.0
	World_Size_Height = 1000.0
	population_size = 400
	hunter_size = 8

)


type Boid struct {
	position pixel.Vec
	velocity pixel.Vec
	last_updated time.Time
	color pixel.RGBA
}


func angle(v1 pixel.Vec, v2 pixel.Vec) float64{
	return math.Acos(v1.Dot(v2) / (math.Sqrt(norm(v1)) * math.Sqrt(norm(v2))))
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
	// take into accound the border, so that the distance in the min between the distance considering borders and the 'normal' distance
	dist_borders := math.Sqrt( ( math.Abs(other.X-World_Size_Width) + vec.X ) * ( math.Abs(other.X-World_Size_Width) + vec.X ) + ( math.Abs(other.Y-World_Size_Height) + vec.Y ) * ( math.Abs(other.Y-World_Size_Height) + vec.Y ))
	dist := math.Sqrt( ( other.X - vec.X ) * ( other.X - vec.X ) + ( other.Y - vec.Y ) * ( other.Y - vec.Y ) )
	return  math.Min(dist, dist_borders)

}

// Find for a boid the nearby other boids
// Takes into account a periphery angle so that the boid un unaware of boids behind him
// Hunters don't have this periphery rule
// Retuns the nearby boids and the nearest boid
func (b *Boid) boids_near (otherBoids []*Boid, distance_min float64, periphery bool) ([]*Boid, *Boid){

	peripheryAngle := math.Pi * 0.75

	boids_near := make([]*Boid, 0)
	var min_dist = 10000000.0
	var minBoid *Boid
	for _, boid := range otherBoids{
		if b != boid{
			curr_distance := b.distance_to(boid)
			if(curr_distance<distance_min){

				diff_vec := b.diff_vector(boid)
				angle := angle(diff_vec, b.velocity)
				if periphery && angle < peripheryAngle{
					boids_near = append(boids_near, boid)
					if curr_distance < min_dist {
						min_dist = curr_distance
						minBoid = boid
					}
				} else {
					boids_near = append(boids_near, boid)
					if curr_distance < min_dist {
						min_dist = curr_distance
						minBoid = boid
					}
				}

			}
		}
	}
	return boids_near, minBoid
}

func (b *Boid) limitVelo(){
	square_speed := norm(b.velocity)
	if square_speed > ( Speed_Limit ) {
		b.velocity = b.velocity.Scaled( Speed_Limit / math.Sqrt( square_speed ) )
	}
}
func limitAcc(v pixel.Vec) pixel.Vec{
	square_acc := norm(v)
	if square_acc > ( Acceleration_Limit ) {
		v = v.Scaled( Acceleration_Limit / math.Sqrt( square_acc ) )
	}
	return v
}


// Updates a boid according to other boids and predators
// Passes the updated boid to outBoid channel
func (b *Boid) updateBoid (otherBoids []*Boid, predators []*Boid,outBoid chan *Boid){
	updatedBoid := new(Boid)
	*updatedBoid = *b
	var acceleration pixel.Vec


	boidsNear, _ := b.boids_near(otherBoids, 200, true)
	huntersNear, _ := b.boids_near(predators, 100, true)

	vec_avoid, vec_align, vec_cohesion, vec_separation := pixel.V(0.0, 0.0), pixel.V(0.0, 0.0),
	pixel.V(0.0, 0.0), pixel.V(0.0, 0.0)
	count_avoid, count_align, count_cohesion, count_separation := 0.0, 0.0, 0.0, 0.0

	// Rules for boids
	for _, bNear := range boidsNear{

		diff_vector := b.diff_vector(bNear)
		//fmt.Println("diff_vector", diff_vector)
		distance := b.distance_to(bNear)

		if distance < Alignment_Distance {
			vec_align = vec_align.Add( bNear.velocity).Scaled(1.0)
			count_align++
		}

		if distance < Cohesion_Distance {
			vec_cohesion = vec_cohesion.Add( bNear.position).Scaled(1.0)
			count_cohesion++
		}

		if distance < Separation_Distance {
			vec_separation = diff_vector.Scaled(3.0/distance)
			count_separation++
		}


	}
	// Rules for predators
	for _, hunterNear := range huntersNear{
		distance := b.distance_to(hunterNear)
		if distance < Avoid_Distance {
			vec_avoid = vec_avoid.Sub( b.diff_vector(hunterNear)).Scaled(0.4/distance)
			count_avoid++
		}
	}

	// limit rules
	vec_separation = limitAcc(vec_separation)
	vec_align = limitAcc(vec_align)
	vec_cohesion = limitAcc(vec_cohesion)
	vec_avoid = limitAcc(vec_avoid)


	// Update Acceleration with resulting rules
	if count_align > 0 {
		vec_align = vec_align.Scaled(1.0/count_align)
		vec_align = vec_align.Sub(b.velocity).Scaled(0.44)
		acceleration = acceleration.Add(vec_align)
	}

	if count_cohesion > 0 {
		vec_cohesion = vec_cohesion.Scaled(1.0/count_cohesion)
		vec_cohesion = vec_cohesion.Sub(b.velocity).Scaled(0.01225)
		acceleration = acceleration.Add(vec_cohesion)
	}

	if count_separation > 0 {
		vec_separation = vec_separation.Scaled(1.0/count_separation)
		vec_separation = vec_separation.Sub(b.velocity).Scaled(0.054)
		acceleration = acceleration.Add(vec_separation)
	}

	if count_avoid > 0 {
		vec_avoid = vec_avoid.Sub(b.velocity).Scaled(60/count_avoid)
		acceleration = acceleration.Sub(vec_avoid)
	}


	acceleration = limitAcc(acceleration)


	updatedBoid.velocity = updatedBoid.velocity.Add( acceleration )
	updatedBoid.limitVelo()
	updatedBoid.position = updatedBoid.position.Add( updatedBoid.velocity)

	// Restrict the bounds
	updatedBoid.restrictBounds()

	outBoid <- updatedBoid
}


func (b *Boid) restrictBounds(){
	for b.position.X < 0 {
		b.position.X += World_Size_Width
	}

	for b.position.X > World_Size_Width {
		b.position.X -= World_Size_Width
	}

	for b.position.Y < 0 {
		b.position.Y += World_Size_Height
	}

	for b.position.Y > World_Size_Height {
		b.position.Y -= World_Size_Height
	}
}


func (b *Boid) updateHunter (otherBoids []*Boid, outBoid chan *Boid) {
	updatedBoid := new(Boid)
	*updatedBoid = *b
	var acceleration pixel.Vec

	boidsNear, nearestBoid := b.boids_near(otherBoids, 300, false)

	vec_attack, vec_cohesion := pixel.V(0.0,0.0), pixel.V(0.0, 0.0)
	count_attack, count_cohesion := 0.0, 0.0


	for _, bNear := range boidsNear{

		distance := b.distance_to(bNear)
		if distance < 50 {
			vec_cohesion = vec_cohesion.Add( bNear.velocity)
			count_cohesion++
		}
	}

	if nearestBoid != nil {
		vec_attack = b.diff_vector(nearestBoid).Scaled(-0.01)
		count_attack ++
	}


	if count_cohesion > 0 {
		vec_cohesion = vec_cohesion.Scaled(1.0/count_cohesion)
		vec_cohesion = vec_cohesion.Sub(b.velocity).Scaled(0.0525)
		acceleration = acceleration.Add(vec_cohesion)
	}

	if count_attack > 0 {
		acceleration = acceleration.Add(vec_attack)
	}

	limitAcc(acceleration)
	updatedBoid.velocity = updatedBoid.velocity.Add( acceleration )
	updatedBoid.limitVelo()
	updatedBoid.position = updatedBoid.position.Add( updatedBoid.velocity)
	updatedBoid.restrictBounds()

	outBoid <- updatedBoid

}




func create_boid (color pixel.RGBA) *Boid {
	new_boid := new( Boid )
	new_boid.position.X = rand.Float64() * World_Size_Width - 400
	new_boid.position.Y = rand.Float64() * World_Size_Height - 400
	new_boid.velocity.X = rand.Float64() * 4 -2
	new_boid.velocity.Y = rand.Float64() * 4 -2
	new_boid.last_updated = time.Now()
	new_boid.color = color

	return new_boid
}

func updateWorker(last_state_len int, new_state []*Boid, boid_updates chan *Boid, wg *sync.WaitGroup){
	for i := 0; i < last_state_len; i++ {
		x, ok := <-boid_updates
		if ok{
			new_state[i] = x
		}
	}
	wg.Done()
}

func drawWorker(last_state *[]*Boid, state_channel chan []*Boid,  win *pixelgl.Window, wg *sync.WaitGroup){
	x, ok := <- state_channel
	if ok {
		*last_state = x
		draw_state( x, win )
	}
	wg.Done()
}



func loop_state(last_state []*Boid,  state_channel chan []*Boid, boid_updates chan *Boid, last_state_hunter []*Boid,  state_channel_hunter chan []*Boid, hunter_updates chan *Boid) {

	new_state := make( []*Boid, population_size )
	new_state_hunter := make([]*Boid, hunter_size)


	for i := 0; i < len( last_state ); i++ {
		go last_state[ i ].updateBoid(last_state, last_state_hunter, boid_updates )
	}

	for i := 0; i < len( last_state_hunter); i++ {
		go last_state_hunter[ i ].updateHunter(last_state, hunter_updates )
	}

	var wg sync.WaitGroup
	wg.Add(2)
	go updateWorker(len(last_state), new_state,  boid_updates, &wg)
	go updateWorker(len(last_state_hunter), new_state_hunter,  hunter_updates, &wg)
	wg.Wait()

	state_channel <- new_state
	state_channel_hunter <- new_state_hunter
}


func run() {
	rand.Seed(time.Now().UnixNano())
	cfg := pixelgl.WindowConfig{
		Title:  "Boids",
		Bounds: pixel.R(0, 0, World_Size_Width, World_Size_Height),
		VSync:  true,
	}
	win, err := pixelgl.NewWindow(cfg)
	if err != nil {
		panic(err)
	}


	last_state_hunter := make([]*Boid, 0)

	for i:=0; i<hunter_size; i++ {
		last_state_hunter = append(last_state_hunter, create_boid(pixel.RGB(0,0,1)))
	}

	state_channel_hunter := make( chan []*Boid )
	hunter_updates := make( chan *Boid, 1 )


	last_state := make([]*Boid, 0)
	for i:=0; i<population_size; i++ {
		last_state = append(last_state, create_boid(pixel.RGB(1,0,0)))
	}
	state_channel := make( chan []*Boid )
	boid_updates := make( chan *Boid, 1 )



	for !win.Closed() {
		win.Clear(colornames.White)
		go loop_state(last_state, state_channel, boid_updates, last_state_hunter, state_channel_hunter, hunter_updates)

		var wg sync.WaitGroup

		wg.Add(2)
		go drawWorker(&last_state, state_channel,  win, &wg)
		go drawWorker(&last_state_hunter, state_channel_hunter,  win, &wg)
		wg.Wait()

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
	vec := b.position.Sub(pixel.V(13, 3))
	imd.Push(vec)
	vec = b.position.Add(pixel.V(-13, 3))
	//imd.Push(pixel.V(600, 600))
	imd.Push(vec)
	mat = mat.Rotated(b.position,  math.Atan2( b.velocity.Y, b.velocity.X ))
	imd.Color = b.color
	imd.SetMatrix(mat)
	imd.Polygon(0)
	imd.Draw(win)

}