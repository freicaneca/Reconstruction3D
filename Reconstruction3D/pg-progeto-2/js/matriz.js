function Matriz(u, v, n) {
    if (!(u instanceof Vetor) ||
	!(v instanceof Vetor) ||
	!(n instanceof Vetor)) {
	console.log('u: ' + JSON.stringify(u));
	console.log('v: ' + JSON.stringify(v));
	console.log('n: ' + JSON.stringify(n));
	throw 'Não é uma matriz';
    }
    this.a = u.x;
    this.b = u.y;
    this.c = u.z;

    this.d = v.x;
    this.e = v.y;
    this.f = v.z;

    this.g = n.x;
    this.h = n.y;
    this.i = n.z;
}

Matriz.prototype.vezesVetor = function(v2) {
    if (!(v2 instanceof Vetor)) {
	console.log('v2: ' + JSON.stringify(v2));
	throw 'v2 não é um vetor';
    }
    var v = new Vetor(this.a * v2.x + this.b * v2.y + this.c * v2.z,
		      this.d * v2.x + this.e * v2.y + this.f * v2.z,
		      this.g * v2.x + this.h * v2.y + this.i * v2.z);
    return v;
};

Matriz.prototype.inversa = function() {
    var det = ((this.a)*(this.e)*(this.i) - (this.a)*(this.f)*(this.h)
	       - (this.b)*(this.d)*(this.i) + (this.b)*(this.f)*(this.g)
	       + (this.c)*(this.d)*(this.h) - (this.c)*(this.e)*(this.g));

    if (det == 0) {
	throw 'Matriz não tem inversa';
    }
    det = 1/det;
    
    var a, b, c, d, e, f, g, h, i;
    a = (((this.e)*(this.i) - (this.f)*(this.h))*det);
    b = (((this.f)*(this.g) - (this.d)*(this.i))*det);
    c = (((this.d)*(this.h) - (this.e)*(this.g))*det);
    
    d = (((this.c)*(this.h) - (this.b)*(this.i))*det);
    e = (((this.a)*(this.i) - (this.c)*(this.g))*det);
    f = (((this.b)*(this.g) - (this.a)*(this.h))*det);
    
    g = (((this.b)*(this.f) - (this.c)*(this.e))*det);
    h = (((this.c)*(this.d) - (this.a)*(this.f))*det);
    i = (((this.a)*(this.e) - (this.b)*(this.d))*det);
    var v1 = new Vetor(a, d, g);
    var v2 = new Vetor(b, e, h);
    var v3 = new Vetor(c, f, i);

    var m = new Matriz(v1, v2, v3);
    return m;
};

/******************/
function vetorMatriz4d(vetor, matriz) {
    return new Ponto(matriz[0][0] * vetor.x + matriz[0][1] * vetor.y +
		     matriz[0][2] * vetor.z + matriz[0][3],
		     matriz[1][0] * vetor.x + matriz[1][1] * vetor.y +
		     matriz[1][2] * vetor.z + matriz[1][3],
		     matriz[2][0] * vetor.x + matriz[2][1] * vetor.y +
		     matriz[2][2] * vetor.z + matriz[2][3]);
}

function vvetorMatriz4d(vetor, matriz) {
    return new Vetor(matriz[0][0] * vetor.x + matriz[0][1] * vetor.y +
		     matriz[0][2] * vetor.z + matriz[0][3],
		     matriz[1][0] * vetor.x + matriz[1][1] * vetor.y +
		     matriz[1][2] * vetor.z + matriz[1][3],
		     matriz[2][0] * vetor.x + matriz[2][1] * vetor.y +
		     matriz[2][2] * vetor.z + matriz[2][3]);
}

function matrizvMatriz4d(matriz1, matriz2) {
    var m = new Array(4);
    for (var i = 0; i < 4; i++) {
	m[i] = new Array(4);
	for (var j = 0; j < 4; j++) {
	    m[i][j] = 0;
	    for (var k = 0; k < 4; k++) {
		m[i][j] += matriz1[i][k] * matriz2[k][j];
	    }
	}
    }
    return m;
}
