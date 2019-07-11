function Leitor(str) {
    if (typeof str !== 'string' || !(str instanceof String)) {
	this.str = str;
	this.contador = 0;
    } else {
	var errormsg = 'str não é string. ' + typeof str;
	throw errormsg;
    }
}

Leitor.prototype.proximaPalavra = function() {
    while (this.contador < this.str.length &&
	   (this.str[this.contador] === ' ' ||
	    this.str[this.contador] === '\n')) {
	this.contador++;
    }
    if (this.contador == this.str.length) {
	throw 'Nenhuma palavra foi encontrada';
    }
    var r = '';
    for (; this.contador < this.str.length; this.contador++) {
	if (this.str[this.contador] === ' ' ||
	    this.str[this.contador] === '\n') {
	    break;
	}
	r = r + this.str[this.contador];
    }
    return r;
};

// ignora todas as palavras até encontrar um inteiro ou percorrer toda a string
Leitor.prototype.proximoInt = function() {
    var r = null;
    try {
	r = parseInt(this.proximaPalavra());
	while (isNaN(r)) {
	    r = parseInt(this.proximaPalavra());
	}
    } catch(err) {
	throw 'Nenhum int foi encontrado';
    }
    return r;
};

// ignora todas as palavras até encontrar um float ou percorrer toda a string
Leitor.prototype.proximoFloat = function() {
    var r = null;
    try {
	r = parseFloat(this.proximaPalavra());
	while (isNaN(r)) {
	    r = parseFloat(this.proximaPalavra());
	}
    } catch(err) {
	throw 'Nenhum float foi encontrado';
    }
    return r;
};

// a primeira palavra encontrada precisa ser um int
Leitor.prototype.proximoInt2 = function() {
    var i = this.contador;
    while (i < this.str.length &&
	   (isNaN(parseInt(this.str[i])) &&
	    this.str[i] !== '-')) {
	i++;
    }
    if (i == this.str.length) {
	throw 'Nenhum int foi encontrado';
    }
    if (this.str[i] == '-' && isNaN(parseInt(this.str[i+1]))) {
	invalidNum = this.str[i] + this.str[i+1];
	throw 'Número inválido encontrado: ' + invalidNum +
	    ', na posição: ' + i;
    }
    this.contador = i;
    r = '';
    for (; this.contador < this.str.length; this.contador++) {
	if (isNaN(parseInt(this.str[this.contador])) &&
	    this.str[this.contador] !== '-') {
	    break;
	}
	r = r + this.str[this.contador];
    }
    return parseInt(r);
};

Leitor.prototype.lerCamera = function() {
    this.contador = 0;
    var C,
	N,
	V,
	h = {},
	d,
	x, y, z;

    try {
	x = this.proximoFloat();
	y = this.proximoFloat();
	z = this.proximoFloat();
	C = new Ponto(x, y, z);
	
	x = this.proximoFloat();
	y = this.proximoFloat();
	z = this.proximoFloat();
	N = new Vetor(x, y, z);
	
	x = this.proximoFloat();
	y = this.proximoFloat();
	z = this.proximoFloat();
	V = new Vetor(x, y, z);

	d = this.proximoFloat();

	h.x = this.proximoFloat();
	h.y = this.proximoFloat();
    } catch (err) {
	throw 'Arquivo não está no formato correto de câmera\n' + err;
    }
    return {C, N, V, d, h};
};

Leitor.prototype.lerIluminacao = function() {
    this.contador = 0;
    var Pl,
	ka,
	Ia,
	kd,
	Od,
	ks,
	Il,
	n,
	x, y, z;
    try {
	x = this.proximoFloat();
	y = this.proximoFloat();
	z = this.proximoFloat();
	Pl = new Ponto(x, y, z);
	
	ka = this.proximoFloat();

	x = this.proximoInt();
	y = this.proximoInt();
	z = this.proximoInt();
	Ia = new Vetor(x, y, z);
	
	kd = this.proximoFloat();

	x = this.proximoFloat();
	y = this.proximoFloat();
	z = this.proximoFloat();
	Od = new Vetor(x, y, z);
	
	ks = this.proximoFloat();

	x = this.proximoInt();
	y = this.proximoInt();
	z = this.proximoInt();
	Il = new Vetor(x, y, z);
	
	n = this.proximoFloat();
    } catch (err) {
	throw 'Arquivo não está no formato correto de iluminação\n' + err;
    }
    return {Pl, ka, Ia, kd, Od, ks, Il, n};
};

// cada vértice é um objeto Ponto,
// cada face é um objeto Triangulo.
Leitor.prototype.lerObjeto = function() {
    this.contador = 0;
    
    // V de vértices, F de faces
    // (notação de sílvio, equivalente a pontos e triângulos)
    var V = [],
	F = [],
	x, y, z;
    try {
	var qtdVertices = this.proximoInt(),
	    qtdFaces = this.proximoInt();
	// ler todos os vertices
	for (var i = 0; i < qtdVertices; i++) {
	    x = this.proximoFloat();
	    y = this.proximoFloat();
	    z = this.proximoFloat();
	    V[i] = new Ponto(x, y, z);
	}
	// agora ler todas as faces
	for (i = 0; i < qtdFaces; i++) {
	    F[i] = {};
	    // subtrair 1 pois no arquivo não tem ponto 0
	    F[i].a = this.proximoInt() - 1;
	    F[i].b = this.proximoInt() - 1;
	    F[i].c = this.proximoInt() - 1;
	}
    } catch (err) {
	throw 'Arquivo não está no formato correto de objeto\n' + err;
    }
    return {V, F};
};
