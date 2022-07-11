var Vector = function(x, y) { this.x = x === 'undefined' ? 0 : x; this.y = y === 'undefined' ? 0 : y; };
Vector.prototype.add = function(v) { return new Vector(this.x + v.x, this.y + v.y); };
Vector.prototype.sub = function(v) { return new Vector(this.x - v.x, this.y - v.y); };
Vector.prototype.mul = function(factor) { return new Vector(this.x * factor, this.y * factor); };
Vector.prototype.div = function(factor) { return new Vector(this.x / factor, this.y / factor); };
Vector.prototype.mag = function() { return Math.sqrt((this.x * this.x) + (this.y * this.y)); };
Vector.prototype.normalise = function(v) { var mag = this.mag(); return new Vector(this.x / mag, this.y / mag); };
Vector.prototype.dist = function(v) { return Math.sqrt((this.x - v.x)*(this.x - v.x) + (this.y - v.y)*(this.y - v.y)); };
Vector.prototype.limit = function(limit) { return this.mag() > limit ? this.normalise().mul(limit) : this; };
//-----------------------------------------------------------------------------------------------
var Boid = function(parent, position, velocity, size, colour) {
  this.position     = new Vector(position.x, position.y);
  this.velocity     = new Vector(velocity.x, velocity.y);
  this.acceleration = new Vector(0, 0);
  this.size         = size;
  this.colour       = colour;
  this.parent       = parent;
};
Boid.prototype.draw = function () {
  if(!this.hide){
    this.parent.ctx.beginPath();
    this.parent.ctx.fillStyle = this.colour;
    this.parent.ctx.globalAlpha = 0.7;
    this.parent.ctx.arc(this.position.x, this.position.y, this.parent.boidRadius * this.size, 0, 2 * Math.PI);
    this.parent.ctx.fill();
  }
};
// Reynold's rules
Boid.prototype.update = function () {
  // if( this.assignedTable === undefined ){
  if( this.parent.scene == 0 ){
    var v1 = this.cohesion()     ;//.mul(1);
    var v2 = this.separation()   .mul(this.parent.options.separationVar);
    var v2a= this.avoidTables()  .mul(10);
    var v3 = this.alignment()    ;//.mul(1);
    var v4 = this.interactivity().mul(1.8);
    var v5 = this.borders()      .mul(3);
    var v6 = this.assignedToTable().mul(25);
    this.applyForce(v1);
    this.applyForce(v2);
    this.applyForce(v2a);
    this.applyForce(v3);
    this.applyForce(v4);
    this.applyForce(v5);
    this.applyForce(v6);
    this.velocity = this.velocity.add(this.acceleration).limit(this.parent.options.speed);
    this.position = this.position.add(this.velocity);
    this.acceleration = new Vector(0, 0);
  }else if( this.parent.scene == 1 || (this.parent.scene == 2 && typeof this.fractallyLevel === 'undefined')){
    var v2 = this.separation() .mul(this.parent.options.separationVar*4);
    var v2a= this.avoidTables().mul(10);
    var v6 = this.assignedToTable().mul(25);
    this.applyForce(v2);
    this.applyForce(v2a);
    this.applyForce(v6);
    this.velocity = this.velocity.add(this.acceleration).limit(this.parent.options.speed);
    // reduce speed if close to the assigned table!
    var d = this.position.dist(this.parent.tables[this.assignedTable].position);
    if     (d< 20){ this.velocity = this.velocity.div(2); }
    else if(d<100){ this.velocity = this.velocity.div(1.2); }
    this.position = this.position.add(this.velocity);
    this.acceleration = new Vector(0, 0);
  }else if( this.parent.scene == 2 && typeof this.fractallyLevel !== 'undefined' ){
    // move to column next to the table if consensus reached for table
    var v6 = this.columnNextToTable().mul(12);
    this.applyForce(v6);
    this.velocity = this.velocity.add(this.acceleration).limit(this.parent.options.speed).div(2);
    this.position = this.position.add(this.velocity);
    this.acceleration = new Vector(0, 0);
  }else if( this.parent.scene == 3 ){
    var v5 = this.borders()       .mul(2.0);
    var v6 = this.assignedToDoor().mul(25.0);
    this.applyForce(v5);
    this.applyForce(v6);
    this.velocity = this.velocity.add(this.acceleration).limit(this.parent.options.speed*40);
    this.position = this.position.add(this.velocity);
    this.acceleration = new Vector(0, 0);
  }
};
// BOIDS FLOCKING RULES
Boid.prototype.cohesion = function () { // Cohesion rule: steer towards average position of local flockmates
  var sum = new Vector(0, 0); // Average flockmate position
  var count = 0; // number of local flockmates
  for(var i = 0; i < this.parent.boids.length; i++) {
    var d = this.position.dist(this.parent.boids[i].position);
    if(d > 0 && d < this.parent.visibleRadius) {
      sum = sum.add(this.parent.boids[i].position);
      count++;
    }
  }
  if(count > 0) {
    sum = sum.div(count);
    sum = this.seek(sum);
    return sum;
  } else {
    return new Vector(0, 0);
  }
};
Boid.prototype.separation = function () { // Separation rule: steer to avoid crowding local flockmates
  var steer = new Vector(0, 0);
  var count = 0;
  // For each boid which is too close, calculate a vector pointing
  // away from it weighted by the distance to it
  for(var i = 0; i < this.parent.boids.length; i++) {
    var d = this.position.dist(this.parent.boids[i].position) - (this.size * this.parent.boidRadius);
    if(d > 0 && d < this.parent.separationDist) {
      var diff = this.position
        .sub(this.parent.boids[i].position)
        .normalise()
        .div(d);
      steer = steer.add(diff);
      count++;
    }
  }
  if(count > 0) { steer = steer.div(count); }
  if(steer.mag() > 0) { // Steering = Desired - Velocity
    steer = steer
      .normalise()
      .mul(this.parent.options.speed)
      .sub(this.velocity)
      .limit(this.parent.maxForce);
  }
  return steer;
};
///////////////////////////////////////////
Boid.prototype.avoidTables = function () {
  var steer = new Vector(0, 0);
  var count = 0;
  for(var i = 0; i < this.parent.tables.length; i++) {
    var d = this.position.dist(this.parent.tables[i].position);
    if(d > 0 && d < 15) {
      var diff = this.position
        .sub(this.parent.tables[i].position)
        .normalise()
        .div(d);
      steer = steer.add(diff);
      count++;
    }
  }
  if(count > 0) { steer = steer.div(count); }
  if(steer.mag() > 0) { // Steering = Desired - Velocity
    steer = steer
      .normalise()
      .mul(this.parent.options.speed)
      .sub(this.velocity)
      .limit(this.parent.maxForce);
  }
  return steer;
};
///////////////////////////////////////////
Boid.prototype.assignedToTable = function () {
  var steer = new Vector(0, 0);
  if( this.assignedTable !== undefined ){
    var d = this.position.dist(this.parent.tables[this.assignedTable].position);
    if(d > 15) {
      var diff = this.parent.tables[this.assignedTable].position
        .sub(this.position)
        .normalise()
        .div(d);
      steer = steer.add(diff);
    }
    if(steer.mag() > 0) { // Steering = Desired - Velocity
      steer = steer
        .normalise()
        .mul(this.parent.options.speed * 2)
        .sub(this.velocity)
        .limit(this.parent.maxForce * 2);
    }
  }
  return steer;
};
Boid.prototype.assignedToDoor = function () {
  var steer = new Vector(0, 0);
  if( this.toDoor !== undefined ){
    var d = this.position.dist(this.toDoor);
    if(d > 15 && !this.hide) {
      var diff = this.toDoor
        .sub(this.position)
        .normalise()
        .div(d);
      steer = steer.add(diff);
    }
    if(Math.abs(this.position.x - this.toDoor.x)<15){
      this.hide = true;
    }
    if(steer.mag() > 0) { // Steering = Desired - Velocity
      steer = steer
        .normalise()
        .mul(this.parent.options.speed*1.4)
        .sub(this.velocity)
        .limit(this.parent.maxForce*1.4);
    }
  }
  return steer;
};
///////////////////////////////////////////
Boid.prototype.columnNextToTable = function () {
  var steer = new Vector(0, 0);
  if( this.assignedTable !== undefined ){
    // TODO: each boid should be attracted to a column near the table
    var diff = this.parent.tables[this.assignedTable].position
      .add(new Vector(15, (this.fractallyLevel-3.5)*8))
      .sub(this.position)
      .normalise();
    steer = steer.add(diff);
    if(steer.mag() > 0) { // Steering = Desired - Velocity
      steer = steer
        .normalise()
        .mul(this.parent.options.speed)
        .sub(this.velocity)
        .limit(this.parent.maxForce);
    }
  }
  return steer;
};
///////////////////////////////////////////
Boid.prototype.alignment = function () { // Alignment rule: steer toward average heading of local flockmates
  var sum = new Vector(0, 0); // Average velocity
  var count = 0; // number of local flockmates
  for(var i = 0; i < this.parent.boids.length; i++) {
    var d = this.position.dist(this.parent.boids[i].position);
    if(d > 0 && d < this.parent.visibleRadius) {
      sum = sum.add(this.parent.boids[i].velocity);
      count++;
    }
  }
  if(count > 0) {
    sum = sum
      .div(count)
      .normalise()
      .mul(this.parent.options.speed);
    var steer = sum.sub(this.velocity);
    steer = steer.limit(this.parent.maxForce);
    return steer;
  } else {
    return new Vector(0, 0);
  }
};
Boid.prototype.interactivity = function () {
  if(this.parent.options.interactive && this.parent.mousePos !== undefined &&
     this.position.dist(this.parent.mousePos) < this.parent.visibleRadius) {
    return this.seek(this.parent.mousePos);
  } else {
    return new Vector(0, 0);
  }
};
Boid.prototype.borders = function() {
  let x=0, y=0;
  if(this.position.x < this.parent.canvas.width  * 0.002) { x =  1; };
  if(this.position.x > this.parent.canvas.width  * 0.998) { x = -1; };
  if(this.position.y < this.parent.canvas.height * 0.004) { y =  1; };
  if(this.position.y > this.parent.canvas.height * 0.996) { y = -1; };
  return new Vector(x, y);
};
Boid.prototype.seek = function(target) { // Calculate a force to apply to a boid to steer it towards a target position
  return target
    .sub(this.position)
    .normalise()
    .mul(this.parent.options.speed)
    .sub(this.velocity)
    .limit(this.parent.maxForce);
};
Boid.prototype.applyForce = function(force) {
  this.acceleration = this.acceleration.add(force.div(this.size));
};
//-----------------------------------------------------------------------------------------------
var Table = function(parent, position) {
  this.position     = new Vector(position.x, position.y);
  this.velocity     = new Vector(0, 0);
  this.acceleration = new Vector(0, 0);
  this.size         = 1;
  this.colour       = '#999';
  this.parent       = parent;
};
Table.prototype.draw = function (color) {
  this.parent.ctx.beginPath();
  this.parent.ctx.fillStyle = typeof color === 'undefied' ? this.colour : color;
  this.parent.ctx.globalAlpha = 0.4;
  this.parent.ctx.arc(this.position.x, this.position.y, this.size * this.parent.tableRadius, 0, 2 * Math.PI);
  this.parent.ctx.fill();
};
Table.prototype.update = function () {
  if( this.parent.scene != 3 ){
    var v2 = this.separation().mul(20.0);
    var v5 = this.borders()   .mul(8.0);
    this.applyForce(v2);
    this.applyForce(v5);
    this.velocity = this.velocity.add(this.acceleration).limit(this.parent.options.tablesSpeed);
    this.position = this.position.add(this.velocity);
    this.acceleration = new Vector(0, 0);
  // }else if( this.parent.scene == 3 ){
  }
};
Table.prototype.separation = function () { // Separation rule: steer to avoid crowding local flockmates
  var steer = new Vector(0, 0);
  var count = 0;
  for(var i = 0; i < this.parent.tables.length; i++) {
    var d = this.position.dist(this.parent.tables[i].position) - (this.size);
    if(d > 0 && d < this.parent.tableSeparationDist) {
      var diff = this.position
        .sub(this.parent.tables[i].position)
        .normalise()
        .div(d);
      steer = steer.add(diff);
      count++;
    }
  }
  if(count > 0) { steer = steer.div(count); }
  if(steer.mag() > 0) { // Steering = Desired - Velocity
    steer = steer
      .normalise()
      .mul(this.parent.options.speed)
      .sub(this.velocity)
      .limit(this.parent.maxForce);
  }
  return steer;
};
Table.prototype.borders = function() {
  let x=0, y=0, factorX = 0.03, factorY = 0.05;
  if(this.position.x < this.parent.canvas.width  * factorX    ) { x =  1; };
  if(this.position.x > this.parent.canvas.width  * (1-factorX)) { x = -1; };
  if(this.position.y < this.parent.canvas.height * factorY    ) { y =  1; };
  if(this.position.y > this.parent.canvas.height * (1-factorY)) { y = -1; };
  return new Vector(x, y);
};
Table.prototype.applyForce = function(force) {
  this.acceleration = this.acceleration.add(force.div(this.size));
};
//-----------------------------------------------------------------------------------------------
var BoidsCanvas = function(canvas, options) {
  this.canvasDiv = canvas;
  this.canvasDiv.size = {
    'width': this.canvasDiv.offsetWidth,
    'height': this.canvasDiv.offsetHeight
  };
  // Set customisable boids parameters
  options = options !== undefined ? options : {};
  this.options = {
    background: (options.background !== undefined) ? options.background : '#1a252f',
    speed: this.setSpeed(options.speed),
    tablesSpeed:7,
    separationVar: 1.5,
    interactive: (options.interactive !== undefined) ? options.interactive : true,
    mixedSizes: (options.mixedSizes !== undefined) ? options.mixedSizes : true,
    boidColours: (options.boidColours !== undefined && options.boidColours.length !== 0) ? options.boidColours : ["#ff3333"]
  };
  // Internal boids parameters
  this.visibleRadius = 150;
  this.maxForce = 0.04;
  this.separationDist = 80;
  this.boidRadius = 3; //size of the smallest boid
  this.tableRadius = 10;
  this.tableSeparationDist = 80;
  this.init();
};
BoidsCanvas.prototype.init = function() {
  this.bgDiv = document.createElement('div');
  this.canvasDiv.appendChild(this.bgDiv);
  this.setStyles(this.bgDiv, { 'position': 'absolute', 'top': 0, 'left': 0, 'bottom': 0, 'right': 0, 'z-index': 1 });
  if ((/(^#[0-9A-F]{6}$)|(^#[0-9A-F]{3}$)/i).test(this.options.background)) {
    this.setStyles(this.bgDiv, { 'background': this.options.background });
  } else if ((/\.(gif|jpg|jpeg|tiff|png)$/i).test(this.options.background)) {
    this.setStyles(this.bgDiv, {
      'background': 'url("' + this.options.background + '") no-repeat center',
      'background-size': 'cover'
    });
  } else {
    console.error('Please specify a valid background image or hexadecimal color');
    return false;
  }
  this.canvas = document.createElement('canvas');
  this.canvasDiv.appendChild(this.canvas);
  this.ctx = this.canvas.getContext('2d');
  this.ctx.font = 'i48px serif';
  this.canvas.width = this.canvasDiv.size.width;
  this.canvas.height = this.canvasDiv.size.height;
  this.setStyles(this.canvasDiv, { 'position': 'relative' });
  this.setStyles(this.canvas, { 'z-index': '20', 'position': 'relative' });
  window.addEventListener('resize', () => {
    if (this.canvasDiv.offsetWidth === this.canvasDiv.size.width && this.canvasDiv.offsetHeight === this.canvasDiv.size.height) { return false; }
    this.canvas.width = this.canvasDiv.size.width = this.canvasDiv.offsetWidth;
    this.canvas.height = this.canvasDiv.size.height = this.canvasDiv.offsetHeight;
    this.initialiseBoids();
  });
  this.initialiseBoids();
  this.startTime = Date.now() / 1000;
  this.canvas.addEventListener('mousemove', function (e) {
    this.mousePos = new Vector(e.clientX - this.canvas.offsetLeft,
                               e.clientY - this.canvas.offsetTop);
  }.bind(this));
  this.canvas.addEventListener('mouseleave', function (e) { this.mousePos = undefined; }.bind(this));
  requestAnimationFrame(this.update.bind(this));
};
BoidsCanvas.prototype.shuffleIndexes = (howMany) => {
  let shuffle = function (array) {
    for (let i = array.length - 1; i > 0; i--) {
      const j = Math.floor(Math.random() * (i + 1));
      [array[i], array[j]] = [array[j], array[i]];
    }
  }
  let arr = Array.from(Array(howMany).keys()); // [ 1 : howMany ]
  shuffle(arr);
  return arr;
}
BoidsCanvas.prototype.initialiseBoids = function() {
  this.boids = [];
  this.boidsPre = [];
  this.scene = 0;
  this.lastFractallyLevelAssignedToBoid = -1;
  let doorPixels = this.canvas.height * 0.20;
  this.totalNumOfPeope = 1105;
  this.shuffledBoids = this.shuffleIndexes(this.totalNumOfPeope);
  for(var i = 0; i < this.totalNumOfPeope; i++) {
    var position = new Vector(Math.floor(this.canvas.width-10+Math.random()*10),
                              Math.floor((this.canvas.height-doorPixels)/2+Math.random()*doorPixels));
    var max_velocity = 5;
    var min_velocity = -5;
    var velocity = new Vector(Math.floor(Math.random()*(0-min_velocity)+min_velocity),
                              Math.floor(Math.random()*(max_velocity-min_velocity+1)+min_velocity));
    var size = (this.options.mixedSizes) ? Math.floor(Math.random()*(3-1+1)+1) : 1;
    var colourIdx = Math.floor(Math.random()*(this.options.boidColours.length-1+1));
    this.boidsPre.push(new Boid(this, position, velocity, size, this.options.boidColours[colourIdx]));
  }
  this.tablesOf5 = this.totalNumOfPeope % 6 == 0 ? 0 : 6 - this.totalNumOfPeope % 6;
  this.tablesOf6 = Math.ceil(this.totalNumOfPeope / 6) - this.tablesOf5;
  this.tablesAll = this.tablesOf5 + this.tablesOf6
  this.tables = [];
  this.tablesPre = [];
  for(let i = 0; i < this.tablesAll; i++){
    let position = new Vector(
      Math.random()*this.canvas.width  * 0.80 + this.canvas.width  * 0.10,
      Math.random()*this.canvas.height * 0.80 + this.canvas.height * 0.10
    );
    this.tablesPre.push(new Table(this, position));
  }
};
BoidsCanvas.prototype.allTablesReachConsensus =function() {
  this.tblBoids = this.boids.reduce((acc, item) => {
    if( typeof (acc[item.assignedTable]) === 'undefined' ){ acc[item.assignedTable] = []; }
    acc[item.assignedTable].push(item);
    return acc;
  }, []);
  for(let i=0; i<this.tblBoids.length; i++){
    for(let j=0; j<this.tblBoids[i].length; j++){
      this.tblBoids[i][j].fractallyLevel = 6-j; // in case of just 5 people, they are levels 2..6
    }
  }
  setTimeout(()=>{
    for(let i=0; i<this.tblBoids.length; i++){
      for(let j=0; j<this.tblBoids[i].length; j++){
        this.tblBoids[i][j].colour = ['#2f9cf4', '#28dbbc', '#8efb53', '#d9df33', '#ff9520', '#ce310d'][6-j];
      }
    }
  }, 1500);
}
BoidsCanvas.prototype.update = function() {
  this.secondsFromStart = Date.now() / 1000 - this.startTime;
  this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
  this.ctx.globalAlpha = 1;
  this.ctx.fillStyle = '#000';
  this.ctx.fillText( (this.secondsFromStart).toFixed(2), 10, 10);
  for (var i = 0; i < this.boids.length; i++) {
    this.boids[i].update();
    this.boids[i].draw();
  }
  if( this.scene == 3 && !this.doorPixels){
    this.tables = [];
    setTimeout(()=>{
      // everybody out except top boids!
      this.doorPixels = this.canvas.height * 0.20;
      for(let i=0; i<this.tblBoids.length; i++){
        for(let j=0; j<this.tblBoids[i].length - 1; j++){
          this.tblBoids[i][j].toDoor = new Vector(
            Math.floor(this.canvas.width-10+Math.random()*10),
            Math.floor((this.canvas.height-this.doorPixels)/2+Math.random()*this.doorPixels)
          );
        }
      }
    }, 500);
  }
  for (var i = 0; i < this.tables.length; i++) {
    this.tables[i].update();
    if( this.secondsFromStart > 34){
      if(i>this.tables.length-6){ this.tables[i].draw('#9B9'); }
      else{ this.tables[i].draw( this.tables[i].colour ); }
    }else{
      this.tables[i].draw(this.tables[i].colour);
    }
  }
  if(this.boidsPre.length>0){
    let batchSize = 0 + Math.random() * 9 * Math.max(0, (0.94+Math.cos( 2 * this.secondsFromStart )));
    for(let i=0; i<batchSize; i++){ if(this.boidsPre.length>0){ this.boids.push( this.boidsPre.pop() ); } }
  }
  if(this.tablesPre.length>0 && this.secondsFromStart > 10){
    let batchTablesSize = Math.random() * 9 * Math.max(.5, (1+Math.cos( 2 * this.secondsFromStart )));
    for(let i=0; i<batchTablesSize; i++){ if(this.tablesPre.length>0){ this.tables.push( this.tablesPre.pop() ); } }
  }
  if(false){}
  else if( this.secondsFromStart > 14 && this.options.separationVar < 4.5){ this.options.separationVar = 4.5; }
  else if( this.secondsFromStart > 11 && this.options.separationVar < 3.0){ this.options.separationVar = 3.0; }
  else if( this.secondsFromStart >  8 && this.options.separationVar < 2.5){ this.options.separationVar = 2.5; }
  else if( this.secondsFromStart >  5 && this.options.separationVar < 2.0){ this.options.separationVar = 2.0; }
  if(false){}
  else if( this.secondsFromStart > 16 && this.options.tablesSpeed > 0){ this.options.tablesSpeed = 0; }
  else if( this.secondsFromStart > 14 && this.options.tablesSpeed > 1){ this.options.tablesSpeed = 1; }
  else if( this.secondsFromStart > 12 && this.options.tablesSpeed > 3){ this.options.tablesSpeed = 3; }
  if(false){}
  else if( this.secondsFromStart > 63 && this.scene == 3  ){ this.scene = 4; }
  else if( this.secondsFromStart > 42 && this.scene == 2  ){ this.scene = 3; }
  else if( this.secondsFromStart > 38 && this.scene == 1  ){ this.scene = 2; this.allTablesReachConsensus(); }
  else if( this.secondsFromStart > 20 && this.scene == 0  ){ this.scene = 1; }
  // assigning to tables only after all entered the conference
  if( this.secondsFromStart > 16 && this.shuffledBoids.length > 0 ){
    let tableNumber = 0;
    let peopleInCurrentTable = 0;
    for(let i = 0; i < this.boids.length; i++){
      this.boids[ this.shuffledBoids[i] ].assignedTable = tableNumber;
      peopleInCurrentTable++;
      if(tableNumber < this.tablesOf6){
        if( peopleInCurrentTable == 6 ){
          tableNumber++;
          peopleInCurrentTable = 0;
        }
      }else{
        if( peopleInCurrentTable == 5 ){
          tableNumber++;
          peopleInCurrentTable = 0;
        }
      }
    }
    this.shuffledBoids = []; // this makes sure we only do this once
  }
  requestAnimationFrame(this.update.bind(this));
};
BoidsCanvas.prototype.setSpeed = (speed) => ({ slow: 1, medium: 2, fast: 3 }[speed]);
BoidsCanvas.prototype.setStyles = function (div, styles) {
  for (var property in styles) {
    div.style[property] = styles[property];
  }
};
