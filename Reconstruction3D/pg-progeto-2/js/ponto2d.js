class Ponto2d {
    constructor(x, y, i) {
	if (typeof x !== 'number' ||
	    typeof y !== 'number') {
	    console.log('x: ' + x + '\n' +
			'y: ' + y);
	    throw 'Não é um ponto';
	}
	this.x = x;
	this.y = y;
	this.originalVertex = i;
    }
    
    menos(p2) {
	var p = new Ponto2d(this.x - p2.x,
			    this.y - p2.y);
	return p;
    }

    static threeDPD(ponto3d, i, width, height) {

	//Para cada ponto do objeto, projete-o para coordenadas de tela 2D, sem descartar os pontos em coordenadas de vista 3D:
	// a linha abaixo gera os pontos 2D parametrizados no intervalo [-1, 1]:
	//P_objeto_tela = ((d/hx)*(P_objeto_vista.x/P_objeto_vista.z), (d/hy)*(P_objeto_vista.y/P_objeto_vista.z))
	var watX = ((window.camera.d/window.camera.h.x)*(ponto3d.x/ponto3d.z));
	var watY = ((window.camera.d/window.camera.h.y)*(ponto3d.y/ponto3d.z));
	// em seguida parametrizamos os pontos para as dimensões da janela (intervalos [0, width] e [0, height]) ,
	// transformando tudo em inteiro, podendo descartar os pontos gerados no intervalo [-1, 1].
	//P_objeto_tela.x = (int)((P_objeto_tela.x + 1) * width / 2)
	//P_objeto_tela.y = (int)((1 - P_objeto_tela.y) * height / 2) 
	watX = Math.floor((watX + 1) * width / 2);
	watY = Math.floor((1 - watY) * height / 2);

	return new Ponto2d(watX, watY, i);
    }

    static sortPoints(array) {
	return array.sort(function(a, b) {
	    if (a.y < b.y) {
		return -1;
	    }
	    if (a.y > b.y) {
		return 1;
	    }
	    // ordenar por x pois empate em y
	    if (a.x < b.x) {
		return -1;
	    }
	    if (a.x > b.x) {
		return 1;
	    }
	    // empate até em x
	    return 0;
	});
    }
}
