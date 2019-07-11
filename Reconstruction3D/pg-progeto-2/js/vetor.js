function Vetor(x, y, z, NoN) {
    if (typeof x !== 'number' ||
	typeof y !== 'number' ||
	typeof z !== 'number') {
	console.log('x: ' + x + '\n' +
		    'y: ' + y + '\n' +
		    'z: ' + z);
	throw 'Não é um vetor';
    }
    this.x = x;
    this.y = y;
    this.z = z;
    if (!NoN) {
	this.norma = Math.sqrt(this.produtoInterno(this));
    }
}

Vetor.prototype.igual = function(v2) {
    if (!(v2 instanceof Vetor)) {
	return false;
    }
    return this.x == v2.x && this.y == v2.y && this.z == v2.z;
};

Vetor.prototype.mais = function(v2) {
    if (!(v2 instanceof Vetor)) {
	console.log('v2: ' + JSON.stringify(v2));
	throw 'v2 não é um vetor';
    }
    var v = new Vetor(this.x + v2.x,
		      this.y + v2.y,
		      this.z + v2.z);
    return v;
};

Vetor.prototype.menos = function(v2) {
    if (!(v2 instanceof Vetor)) {
	console.log('v2: ' + JSON.stringify(v2));
	throw 'v2 não é um vetor';
    }
    var v = new Vetor(this.x - v2.x,
		      this.y - v2.y,
		      this.z - v2.z);
    return v;
};

Vetor.prototype.produtoEscalar = function(escalar) {
    var v = new Vetor(this.x * escalar,
		      this.y * escalar,
		      this.z * escalar);
    return v;
};

Vetor.prototype.produtoComponentes = function(v2, NoN) {
    if (!(v2 instanceof Vetor)) {
	console.log('v2: ' + JSON.stringify(v2));
	throw 'v2 não é um vetor';
    }
    var v = new Vetor(this.x * v2.x,
		      this.y * v2.y,
		      this.z * v2.z,
		      NoN);
    return v;
};

Vetor.prototype.produtoInterno = function(v2) {
    if (!(v2 instanceof Vetor)) {
	console.log('v2: ' + JSON.stringify(v2));
	throw 'v2 não é um vetor';
    }
    var pi = (this.x * v2.x) + (this.y * v2.y) + (this.z * v2.z);
    return pi;
};

Vetor.prototype.produtoVetorial = function(v2) {
    if (!(v2 instanceof Vetor)) {
	console.log('v2: ' + JSON.stringify(v2));
	throw 'v2 não é um vetor';
    }    
    var v = new Vetor(this.y * v2.z - this.z * v2.y,
		      this.z * v2.x - this.x * v2.z,
		      this.x * v2.y - this.y * v2.x);
    return v;
};

Vetor.prototype.projecao = function(v2) {
    if (!(v2 instanceof Vetor)) {
	console.log('v2: ' + JSON.stringify(v2));
	throw 'v2 não é um vetor';
    }
    var proj = v2.produtoEscalar(this.produtoInterno(v2)/v2.produtoInterno(v2));
    return proj;
};

Vetor.prototype.normalizado = function() {
    var v = new Vetor(this.x/this.norma,
		      this.y/this.norma,
		      this.z/this.norma);
    return v;
};
