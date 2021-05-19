//https://p5js.org/examples/hello-p5-flocking.html

var boids = [];
let theta = 0.0;
let sizeTheta = 0.0;

function setup() {
  createCanvas(windowWidth, windowHeight);
  background(252);
  smooth();

  // colorMode(HSB, 60);
  strokeWeight(10);
  // Add an initial set of boids into the system
  for (var i = 0; i < 250; i++) {
    boids[i] = new Boid(width / 2, height / 2);
  }
}

function draw() {
  fill(255, 0, 0);
  stroke(252, 25);
  if (mouseIsPressed) {
    background(252, 0);
    stroke(252, 25);
  } else {
    background(252, 25);
    stroke(252);
  }

  const controls = {
    // The output of the sin() function oscillates smoothly between -1 and 1.
    oscillate: map(sin(theta), -1, 1, 150, 255),
    oscillateRev: map(sin(sizeTheta), -1, 1, 255, 150),
    oscillateSize: map(sin(sizeTheta), -1, 1, 2, 10),
  };

  // With each cycle, increment theta
  theta += 0.05;
  sizeTheta += 0.05;
  // strokeWeight(controls.oscillateSize / 2);
  // Run all the boids
  for (var i = 0; i < boids.length; i++) {
    boids[i].run(boids, controls);
  }
}

// Boid class
// Methods for Separation, Cohesion, Alignment added
function Boid(x, y) {
  this.acceleration = createVector(0, 0);
  this.velocity = p5.Vector.random2D();
  this.position = createVector(x, y);
  this.r = 3.0;
  this.maxspeed = 3; // Maximum speed
  this.maxforce = 0.25; // Maximum steering force
}

Boid.prototype.run = function (boids, controls) {
  this.flock(boids, controls);
  this.update();
  this.borders();
  this.render(controls);
};

// Forces go into acceleration
Boid.prototype.applyForce = function (force) {
  this.acceleration.add(force);
};

// We accumulate a new acceleration each time based on three rules
Boid.prototype.flock = function (boids, controls) {
  var sep = this.separate(boids, controls); // Separation
  var ali = this.align(boids, controls); // Alignment
  var coh = this.cohesion(boids, controls); // Cohesion
  // Arbitrarily weight these forces
  sep.mult(2.5);
  ali.mult(1.0);
  coh.mult(1.0);
  // Add the force vectors to acceleration
  this.applyForce(sep);
  this.applyForce(ali);
  this.applyForce(coh);
};

// Method to update location
Boid.prototype.update = function () {
  // Update velocity
  this.velocity.add(this.acceleration);
  // Limit speed
  this.velocity.limit(this.maxspeed);
  this.position.add(this.velocity);
  // Reset acceleration to 0 each cycle
  this.acceleration.mult(0);
};

// A method that calculates and applies a steering force towards a target
// STEER = DESIRED MINUS VELOCITY
Boid.prototype.seek = function (target) {
  var desired = p5.Vector.sub(target, this.position); // A vector pointing from the location to the target
  // Normalize desired and scale to maximum speed
  desired.normalize();
  desired.mult(this.maxspeed);
  // Steering = Desired minus Velocity
  var steer = p5.Vector.sub(desired, this.velocity);
  steer.limit(this.maxforce); // Limit to maximum steering force
  return steer;
};

// Draw boid as a circle
Boid.prototype.render = function (controls) {
  ellipse(
    this.position.x,
    this.position.y,
    controls.oscillateSize * 2,
    controls.oscillateSize * 2
  );
};

// Wraparound
Boid.prototype.borders = function () {
  if (this.position.x < -this.r) this.position.x = width + this.r;
  if (this.position.y < -this.r) this.position.y = height + this.r;
  if (this.position.x > width + this.r) this.position.x = -this.r;
  if (this.position.y > height + this.r) this.position.y = -this.r;
};

// Separation
// Method checks for nearby boids and steers away
Boid.prototype.separate = function (boids, controls) {
  // With each cycle, increment theta

  var desiredseparation = controls.oscillateSize; //5.0;
  var steer = createVector(0, 0);
  var count = 0;
  // For every boid in the system, check if it's too close
  for (var i = 0; i < boids.length; i++) {
    var d = p5.Vector.dist(this.position, boids[i].position);
    // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if (d > 0 && d < desiredseparation) {
      // Calculate vector pointing away from neighbor
      var diff = p5.Vector.sub(this.position, boids[i].position);
      diff.normalize();
      diff.div(d); // Weight by distance
      steer.add(diff);
      count++; // Keep track of how many
    }
  }
  // Average -- divide by how many
  if (count > 0) {
    steer.div(count);
  }

  // As long as the vector is greater than 0
  if (steer.mag() > 0) {
    // Implement Reynolds: Steering = Desired - Velocity
    steer.normalize();
    steer.mult(this.maxspeed);
    steer.sub(this.velocity);
    steer.limit(this.maxforce);
  }
  return steer;
};

// Alignment
// For every nearby boid in the system, calculate the average velocity
Boid.prototype.align = function (boids, controls) {
  // With each cycle, increment theta

  var neighbordist = controls.oscillateSize; //10;
  var sum = createVector(0, 0);
  var count = 0;
  for (var i = 0; i < boids.length; i++) {
    var d = p5.Vector.dist(this.position, boids[i].position);
    if (d > 0 && d < neighbordist) {
      sum.add(boids[i].velocity);
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    sum.normalize();
    sum.mult(this.maxspeed);
    var steer = p5.Vector.sub(sum, this.velocity);
    steer.limit(this.maxforce);
    return steer;
  } else {
    return createVector(0, 0);
  }
};

// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
Boid.prototype.cohesion = function (boids, controls) {
  // With each cycle, increment theta

  // With each cycle, increment theta

  var neighbordist = controls.oscillateSize * 10;
  //var neighbordist = 50;
  var sum = createVector(0, 0); // Start with empty vector to accumulate all locations
  var count = 0;
  for (var i = 0; i < boids.length; i++) {
    var d = p5.Vector.dist(this.position, boids[i].position);
    if (d > 0 && d < neighbordist) {
      sum.add(boids[i].position); // Add location
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    return this.seek(sum); // Steer towards the location
  } else {
    return createVector(0, 0);
  }
};
