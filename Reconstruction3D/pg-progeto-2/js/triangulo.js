function Triangulo(Pa, Pb, Pc, a, b, c) {
    if (!(Pa instanceof Ponto) ||
	!(Pb instanceof Ponto) ||
	!(Pc instanceof Ponto)) {
	console.log('Pa: ' + JSON.stringify(Pa) + '\n' +
		    'Pb: ' + JSON.stringify(Pb) + '\n' +
		    'Pc: ' + JSON.stringify(Pc));
	throw 'Não são pontos';
    }
    // const ab = Pa.menos(Pb);
    // const ac = Pa.menos(Pc);
    // const bc = Pb.menos(Pc);

    // if ((ab.norma + ac.norma <= bc.norma) ||
    // 	(ab.norma + bc.norma <= ac.norma) ||
    // 	(ac.norma + bc.norma <= ab.norma)) {
    // 	console.log('Pa: ' + JSON.stringify(Pa) + '\n' +
    // 		    'Pb: ' + JSON.stringify(Pb) + '\n' +
    // 		    'Pc: ' + JSON.stringify(Pc));
    // 	console.log('ab: ' + ab.norma + '\n' +
    // 		    'ac: ' + ac.norma + '\n' +
    // 		    'bc: ' + bc.norma);
	
    // 	throw 'Pontos não formam um triângulo:\n' +
    // 	    'ab: ' + ab.norma + '\n' +
    // 	    'ac: ' + ac.norma + '\n' +
    // 	    'bc: ' + bc.norma;
    // }
    // triângulo guarda apenas o número dos vértices que compõem ele
    // assim, quando um ponto mudar de coordenada, não preciamos mudar aqui.
    this.a = a;
    this.b = b;
    this.c = c;
    // não sei a necessidade de ter o comprimento dos lados ainda

    // normal do triângulo
    this.N = (Pc.menos(Pa))
	.produtoVetorial(Pb.menos(Pa));
    this.N = (this.N).normalizado();
}
