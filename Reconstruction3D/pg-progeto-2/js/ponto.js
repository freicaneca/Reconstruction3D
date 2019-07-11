function Ponto(x, y, z, NoN) {
    // if (typeof x !== 'number' ||
    // 	typeof y !== 'number' ||
    // 	typeof z !== 'number') {
    // 	console.log('x: ' + x + '\n' +
    // 		    'y: ' + y + '\n' +
    // 		    'z: ' + z);
    // 	throw 'Não é um ponto';
    // }
    this.x = x;
    this.y = y;
    this.z = z;
    if (!NoN) {
	this.N = new Vetor(0, 0, 0);
    }
}

Ponto.prototype.menos = function(pv2, NoN) {
    if (pv2 instanceof Ponto) {
	var v = new Vetor(this.x - pv2.x,
			  this.y - pv2.y,
			  this.z - pv2.z,
			  NoN);
	return v;
    } else if (pv2 instanceof Vetor) {
	// mesma coisa que ponto + (-v)
	var p = new Ponto(this.x - pv2.x,
			  this.y - pv2.y,
			  this.z - pv2.z);
	return p;
    } else {
	console.log('pv2: ' + JSON.stringify(pv2));
	throw 'pv2 não é um ponto ou vetor';
    }
};

Ponto.prototype.mais = function(v2) {
    if (!(v2 instanceof Vetor)) {
	console.log('v2: ' + JSON.stringify(v2));
	throw 'v2 não é um vetor';
    }
    var p = new Ponto(this.x + v2.x,
		      this.y + v2.y,
		      this.z + v2.z);
    return p;
};

Ponto.prototype.distancia = function(p2) {
    if (!(p2 instanceof Ponto)) {
	console.log('p2: ' + JSON.stringify(p2));
	throw new 'p2 não é um ponto';
    }
    const dx = this.x - p2.x;
    const dy = this.y - p2.y;
    return Math.sqrt(dx * dx + dy * dy);
};

Ponto.prototype.vezes = function(escalar) {
    var p = new Ponto(this.x + escalar,
		      this.y + escalar,
		      this.z + escalar);
    return p;
}
