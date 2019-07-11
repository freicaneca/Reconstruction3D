window.onload = function () { 
    // Checar suporte do browser para File
    if (window.File && window.FileReader && window.FileList && window.Blob) {
	var cameraFileChooser = document.getElementById('cameraFile');
	var lightFileChooser = document.getElementById('lightFile');
	var objectFileChooser = document.getElementById('objectFile');
	var botao = document.getElementById('initialSteps');
	cameraFileChooser.addEventListener('change',
					   fileReadingRoutine,
					   false);
	lightFileChooser.addEventListener('change',
					  fileReadingRoutine,
					  false);
	objectFileChooser.addEventListener('change',
					   fileReadingRoutine,
					   false);
	botao.addEventListener('click',
			       initialSteps,
			       false);
    } 
    else { 
	alert("Este navegador não suporta Files");
    } 
};

function leArquivos() {
    var leitor;
    try {
	leitor = new Leitor(window.camFileTxt);
	var values;

	window.camera = null;
	values = leitor.lerCamera();
	window.camera = new Camera(values);
	// Matriz de Mudanca de Base pra camera
	window.MMBcamera = new Matriz((window.camera).U,
				      (window.camera).V,
				      (window.camera).N);
	// output(syntaxHighlight(
	//     JSON.stringify(window.camera, null, 4)),
	//        'chosencamera');
    } catch (err) {
	window.alert(err);
    }
    try {
	leitor = new Leitor(window.lightFileTxt);
	window.iluminacao = null;
	window.iluminacao = leitor.lerIluminacao();
	// output(syntaxHighlight(
	//     JSON.stringify(window.iluminacao, null, 4)
	// ), 'chosenlight');
    } catch (err) {
	window.alert(err);
    }
    try {
	leitor = new Leitor(window.objFileTxt);
	window.objeto = null;
	window.objeto = leitor.lerObjeto();
	// output(syntaxHighlight(
	//     JSON.stringify(window.objeto, null, 4)),
	//        'chosenobject');
    } catch (err) {
	window.alert(err);
    }

}

function initialSteps() {
    leArquivos();

    var Pl_vista, P_objeto_vista;
    // transforma a posição da luz para coordenadas da câmera
    Pl_vista = (window.MMBcamera).vezesVetor(
	((window.iluminacao).Pl).menos((window.camera).C));
    window.iluminacao.Pl = new Ponto(Pl_vista.x, Pl_vista.y, Pl_vista.z);

    // transforma vértices do objeto para coordenadas da câmera
    for (var i = 0; i < (window.objeto.V.length); i++) {
	P_objeto_vista = (window.MMBcamera).vezesVetor(
	    (window.objeto.V[i]).menos(window.camera.C));
	window.objeto.V[i] = new Ponto(P_objeto_vista.x,
				       P_objeto_vista.y,
				       P_objeto_vista.z);
	// debugger;
	// // e aplicando uma caralhinha!
	// var cosine = Math.cos(Math.PI / 30);
	// var sine = Math.cos(Math.PI / 30);
	// window.objeto.V[i] = vetorMatriz4d(
	//     window.objeto.V[i],new Matriz(new Vetor(cosine, -sine, 0),
	// 				  new Vetor(sine, cosine, 0),
	// 				  new Vetor(0, 0, 1)));

    }

    for (i = 0; i < (window.objeto.F.length); i++) {
	// criando triângulos e suas normais
	var a, b, c;
	// debugger;
	a = window.objeto.F[i].a;
	b = window.objeto.F[i].b;
	c = window.objeto.F[i].c;
	window.objeto.F[i] = new Triangulo(window.objeto.V[a],
					   window.objeto.V[b],
					   window.objeto.V[c],
					   a, b, c);
	if (Math.abs(1-window.objeto.F[i].N) >= 0.000000001) {
	    window.alert(window.objeto.F[i].n + ' !!! ' + i);
	}
	// adicionando a normal dos triângulos a cada vértice daquele triângulo
	window.objeto.V[a].N = (window.objeto.V[a].N)
	    .mais(window.objeto.F[i].N);
	window.objeto.V[b].N = (window.objeto.V[b].N)
	    .mais(window.objeto.F[i].N);
	window.objeto.V[c].N = (window.objeto.V[c].N)
	    .mais(window.objeto.F[i].N);
    }

    for (i = 0; i < (window.objeto.V.length); i++) {
	// normalizando as normais dos vértices
	// debugger;
	window.objeto.V[i].N = (window.objeto.V[i].N).normalizado();

    }
    console.log('ok');

    // output(syntaxHighlight(
    // 	JSON.stringify(window.iluminacao, null, 4)), 'chosenlight');
    // output(syntaxHighlight(
    // 	JSON.stringify(window.objeto, null, 4)), 'chosenobject');

    if (window.myp5) {
	window.myp5.remove();
    }
    window.myp5 = new p5(s);
    // projetar pontos pra coordenadas da tela
    // window.ponto2d = createArray(window.objeto.V.length);
    // for (i = 0; i < (window.objeto.V.length); i++) {
    // 	window.ponto2d[i] = Ponto2d.threeDPD(window.objeto.V[i], i,
    // 					     1024, 768);
    // }
    
    // //window.ponto2d = Ponto2d.sortPoints(window.ponto2d);
    // // vou fazer sort por faces para visualizar por enquanto
    // window.ponto2d = window.ponto2d.sort(function(a, b) {
    // 	if (a.originalVertex < b.originalVertex) {
    // 	    return -1;
    // 	}
    // 	if (a.originalVertex > b.originalVertex) {
    // 	    return 1;
    // 	}
    // 	return 0;
    // });
    // drawy();
}

function drawy() {
    var canvas = document.getElementById('cv');
    var ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, ctx.width, ctx.height);
    var points = window.ponto2d;
    var faces = window.objeto.F;
    ctx.beginPath();
    for (var i = 0; i < 100; i++) {
	// de a pra b
     	ctx.moveTo(points[faces[i].a].x, points[faces[i].a].y);
     	ctx.lineTo(points[faces[i].b].x, points[faces[i].b].y);
     	ctx.stroke();
	// 	// de b pra c
	// 	ctx.moveTo(points[faces[i].b].x, points[faces[i].b].y);
	// 	ctx.lineTo(points[faces[i].c].x, points[faces[i].c].y);
	// 	ctx.stroke();
	// 	// de c pra a
	// 	ctx.moveTo(points[faces[i].c].x, points[faces[i].c].y);
	// 	ctx.lineTo(points[faces[i].a].x, points[faces[i].a].y);
	// 	ctx.stroke();
    }
    window.requestAnimationFrame(drawy);
}

function fileReadingRoutine(evt) {
    if (window.myp5) {
	window.myp5.remove();
    }
    var id = evt.target.id;
    // Pegar o objeto que representa o arquivo escolhido
    var fileTobeRead = document.getElementById(id).files[0];

    // Inicializar o FileReader para ler o arquivo
    var fileReader = new FileReader();

    // Esta função deverá rodar uma vez que o fileReader carregar
    // ela guarda o arquivo, para facilitar recliques do botão
    fileReader.onload = function (e) {
	if (id === 'cameraFile') {
	    window.camFileTxt = fileReader.result;
	} else if (id === 'lightFile') {
	    window.lightFileTxt = fileReader.result;
	} else if (id === 'objectFile') {
	    window.objFileTxt = fileReader.result;
	}
    };
    
    // lê o arquivo como texto, eventualmente invocando a função acima
    fileReader.readAsText(fileTobeRead);
}

function syntaxHighlight(json) {
    json = json.replace(/&/g, '&amp;').replace(/</g, '&lt;')
	.replace(/>/g, '&gt;');
    return json.replace(/("(\\u[a-zA-Z0-9]{4}|\\[^u]|[^\\"])*"(\s*:)?|\b(true|false|null)\b|-?\d+(?:\.\d*)?(?:[eE][+\-]?\d+)?)/g, function (match) {
        var cls = 'number';
        if (/^"/.test(match)) {
            if (/:$/.test(match)) {
                cls = 'key';
            } else {
                cls = 'string';
            }
        } else if (/true|false/.test(match)) {
            cls = 'boolean';
        } else if (/null/.test(match)) {
            cls = 'null';
        }
        return '<span class="' + cls + '">' + match + '</span>';
    });
}

function output(inp, id) {
    var div = document.getElementById(id);
    div.innerHTML = '';
    div.appendChild(document.createElement('pre'))
	.innerHTML = inp;
}

function createArray(length) {
    var arr = new Array(length || 0),
        i = length;

    if (arguments.length > 1) {
        var args = Array.prototype.slice.call(arguments, 1);
        while(i--) arr[length-1 - i] = createArray.apply(this, args);
    }

    return arr;
}

var s = function(p) {

    p.setup = function() {
        p.createCanvas(1024, 768);

	// projetar pontos pra coordenadas da tela
	window.ponto2d = createArray(window.objeto.V.length);
	for (i = 0; i < (window.objeto.V.length); i++) {
	    window.ponto2d[i] = Ponto2d.threeDPD(window.objeto.V[i], i,
						 p.width, p.height);
	}

	// projetar luz em coord de tela pra debugar hehe :)
	window.luz2d = Ponto2d.threeDPD(window.iluminacao.Pl, 0,
					p.width, p.height);

	window.zBuffer = createArray(p.width, p.height);
	for (var i = 0; i < window.zBuffer.length; i++) {
	    window.zBuffer[i] = window.zBuffer[i].fill(Infinity);
	}
	renderObj();
    };

    p.draw = function() {
	// var fps = p.frameRate();
	// p.clear();
	// window.zBuffer = createArray(p.width, p.height);
	// for (var i = 0; i < window.zBuffer.length; i++) {
	//     window.zBuffer[i] = window.zBuffer[i].fill(Infinity);
	// }
	// p.fill(255);
	// p.stroke(0);
	// p.text("FPS: " + fps.toFixed(2), 10, 10);
	// renderObj();
	// p.stroke(255, 0, 255);
	// p.line(window.luz2d.x, window.luz2d.y, window.ponto2d[0].x, window.ponto2d[0].y);
	// p.line(window.luz2d.x, window.luz2d.y, window.ponto2d[1].x, window.ponto2d[1].y);
	// p.line(window.luz2d.x, window.luz2d.y, window.ponto2d[2].x, window.ponto2d[2].y);
	// p.line(window.luz2d.x, window.luz2d.y, window.ponto2d[3].x, window.ponto2d[3].y);
	// debugger;
	// // e aplicando uma caralhinha!
	// var cosine = Math.cos(Math.PI / 60);
	// var sine = Math.sin(Math.PI / 60);
	// var matRot = new Matriz(new Vetor(cosine, sine, 0),
	// 			new Vetor(-sine, cosine, 0),
	// 			new Vetor(0, 0, 1));
	// window.ponto2d = createArray(window.objeto.V.length);
	// for (i = 0; i < (window.objeto.V.length); i++) {
	//     var N = window.objeto.V[i].N;
	//     window.objeto.V[i] = vetorMatriz4d(window.objeto.V[i], matRot);
	//     window.objeto.V[i].N = vvetorMatriz4d(N, matRot);
	//     window.ponto2d[i] = Ponto2d.threeDPD(window.objeto.V[i], i,
	// 					 p.width, p.height);
	// }

    };

    function zBufHorizontal(xf, scanlineY, v1, v2, a, b, P4, N4) {
	var lp = lerp(xf, v1, v2);
	if (typeof a == 'undefined' || typeof b == 'undefined') {
	    // debugger;
	}
	var P, N;
	var objVa, objVb;
	if (!a && a != 0) {
	    objVa = P4;
	    objVa.N = N4;
	} else {
	    objVa = window.objeto.V[a];
	}
	if (!b && b != 0) {
	    objVb = P4;
	    objVb.N = N4;
	} else {
	    objVb = window.objeto.V[b];
	}
	if (isNaN(lp.v)) {
	    if (objVa.z <= objVb.z) {
		P = new Ponto(objVa.x,
			      objVa.y,
			      objVa.z);
		N = new Vetor(objVa.N.x,
			      objVa.N.y,
			      objVa.N.z);
	    } else {
		P = new Ponto(objVb.x,
			      objVb.y,
			      objVb.z);
		N = new Vetor(objVb.N.x,
			      objVb.N.y,
			      objVb.N.z);
	    }
	} else {
	    P = new Ponto(objVa.x * lp.u +
			  objVb.x * lp.v,
			  objVa.y * lp.u +
			  objVb.y * lp.v,
			  objVa.z * lp.u +
			  objVb.z * lp.v);
	    N = new Vetor(objVa.N.x * lp.u +
			  objVb.N.x * lp.v,
			  objVa.N.y * lp.u +
			  objVb.N.y * lp.v ,
			  objVa.N.z * lp.u +
			  objVb.N.z * lp.v);
	}
	if (xf >= 0 && scanlineY >= 0 &&
	    xf < p.width && scanlineY < p.height &&
	    P.z <= window.zBuffer[xf][scanlineY]) {
	    if (P.z <= -1000) {
		debugger;
	    }
	    window.zBuffer[xf][scanlineY] = P.z;

	    phong(N, P, xf, scanlineY);
	}
    }

    function phong(N, P, xf, y) {

	N = N.normalizado();
	var L = window.iluminacao.Pl.menos(P).normalizado();
	var V = new Vetor(-P.x, -P.y, -P.z).normalizado();
	var R = (N.produtoEscalar(((N.produtoInterno(L)) * 2))).menos(L);
	R = R.normalizado();
	var ka = window.iluminacao.ka;
	var ks = window.iluminacao.ks;
	var kd = window.iluminacao.kd;
	var Ia = window.iluminacao.Ia;
	var Od = window.iluminacao.Od;
	var Il = window.iluminacao.Il;
	var n = window.iluminacao.n;
	if (V.produtoInterno(N) < 0) {
	    N = N.produtoEscalar(-1);
	}
	var cosLN = L.produtoInterno(N);
	var cosRV = R.produtoInterno(V);
	if (L.produtoInterno(N) < 0) {
	    kd = 0;
	    ks = 0;
	}
	if (R.produtoInterno(V) < 0) {
	    ks = 0;
	}

	var OdIl = Od.produtoComponentes(Il);
	// var I = Ia.produtoEscalar(ka)
	//     .mais((Od.produtoEscalar(L.produtoInterno(N) * kd)).produtoComponentes(Il))
	//     .mais(Il.produtoEscalar(Math.pow(R.produtoInterno(V), n) * ks));

	var Iamb = Ia.produtoEscalar(ka);

	var Id = OdIl.produtoEscalar(kd * cosLN);
	var Is = Il.produtoEscalar(ks * Math.pow(cosRV, n));


	var I = Iamb.mais(Id).mais(Is);
	I.x = Math.min(I.x, 255);
	I.y = Math.min(I.y, 255);
	I.z = Math.min(I.z, 255);

	//	p.stroke(N.z < 0 ? 0 : 128);
	//	p.stroke(Math.floor(I.x*0.2989) + Math.floor(I.y*0.5870) + Math.floor(I.z*0.1140));
//	p.stroke(Math.floor(Math.abs(N.x) * 255), Math.floor(Math.abs(N.y) * 255), Math.floor(Math.abs(N.z) * 255));
//	p.fill(0,0,0,);
//	p.ellipse(xf,y, 4, 4);
	p.stroke(Math.floor(I.x), Math.floor(I.y), Math.floor(I.z));
	p.point(xf, y);
	if (Math.random() < 0.005) {
	    // drawArrow(xf, y, P,N);
	}

    }

    function zBufVertical(xf, y, v1, v2, a, b, P4, N4) {
	var lp = lerp(y, v1, v2);
	if (typeof a == 'undefined' || typeof b == 'undefined') {
	    // debugger;
	}
	var P, N;
	var objVa, objVb;
	if (!a && a != 0) {
	    objVa = P4;
	    objVa.N = N4;
	} else {
	    objVa = window.objeto.V[a];
	}
	if (!b && b != 0) {
	    objVb = P4;
	    objVb.N = N4;
	} else {
	    objVb = window.objeto.V[b];
	}
	if (isNaN(lp.v)) {
	    if (objVa.z <= objVb.z) {
		P = new Ponto(objVa.x,
			      objVa.y,
			      objVa.z);
		N = new Vetor(objVa.N.x,
			      objVa.N.y,
			      objVa.N.z);
	    } else {
		P = new Ponto(objVb.x,
			      objVb.y,
			      objVb.z);
		N = new Vetor(objVb.N.x,
			      objVb.N.y,
			      objVb.N.z);
	    }
	} else {
	    P = new Ponto(objVa.x * lp.u +
			  objVb.x * lp.v,
			  objVa.y * lp.u +
			  objVb.y * lp.v,
			  objVa.z * lp.u +
			  objVb.z * lp.v);
	    N = new Vetor(objVa.N.x * lp.u +
			  objVb.N.x * lp.v,
			  objVa.N.y * lp.u +
			  objVb.N.y * lp.v ,
			  objVa.N.z * lp.u +
			  objVb.N.z * lp.v);
	}
	if (xf >= 0 && y >= 0 &&
	    xf < p.width && y < p.height &&
	    P.z <= window.zBuffer[xf][y]) {
	    if (P.z <= -1000) {
		debugger;
	    }
	    window.zBuffer[xf][y] = P.z;
	    phong(N, P, xf, y);
	}
    }

    function zBuf(xf, scanlineY, v1, v2, v3, a, b, c, P4, N4) {
	var bar = baricentro(new Ponto2d(xf, scanlineY),
			     v1,
			     v2,
			     v3);
	var P, N;
	var objVa, objVb, objVc;
	if (a === null) {
	    objVa = P4;
	    objVa.N = N4;
	} else {
	    objVa = window.objeto.V[a];
	}
	if (b === null) {
	    objVb = P4;
	    objVb.N = N4;
	} else {
	    objVb = window.objeto.V[b];
	}
	if (c === null) {
	    objVc = P4;
	    objVc.N = N4;
	} else {
	    objVc = window.objeto.V[c];
	}
	
	// alfa*v1_3d + beta*v2_3d + gamma*v3_3d
	P = new Ponto(objVa.x * bar.u +
		      objVb.x * bar.v +
		      objVc.x * bar.w,
		      objVa.y * bar.u +
		      objVb.y * bar.v +
		      objVc.y * bar.w,
		      objVa.z * bar.u +
		      objVb.z * bar.v +
		      objVc.z * bar.w,
		      true);
	N = new Vetor(objVa.N.x * bar.u +
	    	      objVb.N.x * bar.v +
	    	      objVc.N.x * bar.w,
	    	      objVa.N.y * bar.u +
	    	      objVb.N.y * bar.v +
	    	      objVc.N.y * bar.w,
	    	      objVa.N.z * bar.u +
	    	      objVb.N.z * bar.v +
	    	      objVc.N.z * bar.w);
	
	

	if (xf >= 0 && scanlineY >= 0 &&
	    xf < p.width && scanlineY < p.height &&
	    P.z <= window.zBuffer[xf][scanlineY]) {
	    if (P.z <= -1000) {
		debugger;
	    }
	    window.zBuffer[xf][scanlineY] = P.z;
	    phong(N, P, xf, scanlineY);
	}// else debugger;

    }

    function fillBottomFlatTriangle(v1, v2, v3, a, b, c, P, N) {
	var invslope1 = (v2.x - v1.x) / (v2.y - v1.y);
	var invslope2 = (v3.x - v1.x) / (v3.y - v1.y);

	var curx1 = v1.x;
	var curx2 = v1.x;
	for (var scanlineY = v1.y; scanlineY <= v2.y && scanlineY >= 0; scanlineY++) {
	    // scanline = 121
	    if (scanlineY >= 124) debugger;
	    var x = curx1 < curx2 ? curx1 : curx2;
	    var span = Math.min(Math.abs(curx1 - curx2), p.width);
	    for (var i = 0; i < span; i++, x++) {
		var xf = Math.floor(x);
		if (xf < p.width && scanlineY < p.height &&
		    xf >= 0 && scanlineY >= 0) {
		    zBuf(xf, scanlineY, v1, v2, v3, a, b, c, P, N);
		}
	    }
	    curx1 += invslope1;
	    curx2 += invslope2;

	}
    }

    function lerp(p, a, b) {
	var v = Math.abs((p-b)/(a-b));
	// if ((a-b) == 0) {
	//     v = 0;
	// }
	var u = 1 - v;
	return {u, v};
    }

    function lerp2d(p, a, b) {
	// u = vol1(p,b)/vol1(a,b)
	var v = Math.sqrt((p.x-b.x)*(p.x-b.x) + (p.y-b.y)*(p.y-b.y)) /
	    Math.sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
	// if (isNaN(u)) {
	//     u = 0;
	// }
	var u = 1 - v;
	return {u, v};
    }

    // encontra as coordenadas baricentricas
    function baricentro(p, a, b, c) {
	// debugger;
	var v0 = b.menos(a), v1 = c.menos(a), v2 = p.menos(a);
	var invden = 1 / (v0.x * v1.y - v1.x * v0.y);
	var v = (v2.x * v1.y - v1.x * v2.y) * invden;
	var w = (v0.x * v2.y - v2.x * v0.y) * invden;
	var u = 1 - v - w;
	return {u, v, w};
    }
    
    function fillTopFlatTriangle(v1, v2, v3, a, b, c, P, N) {
	var invslope1 = (v3.x - v1.x) / (v3.y - v1.y);
	var invslope2 = (v3.x - v2.x) / (v3.y - v2.y);

	var curx1 = v3.x;
	var curx2 = v3.x;
	for (var scanlineY = v3.y; scanlineY > v1.y && scanlineY <= p.width; scanlineY--) {
	    if (scanlineY >= 124) debugger;
	    var x = curx1 < curx2 ? curx1 : curx2;
	    var span = Math.min(Math.abs(curx1 - curx2), p.width);
	    for (var i = 0; i < span; i++, x++) {
		var xf = Math.floor(x);
		if (xf < p.width && scanlineY < p.height &&
		    xf >= 0 && scanlineY >= 0) {
		    zBuf(xf, scanlineY, v1, v2, v3, a, b, c, P, N);
		}
	    }
	    curx1 -= invslope1;
	    curx2 -= invslope2;

	}
    }

    function drawTriangle(triangle) {
	/* at first sort the three vertices by y-coordinate ascending so v1 is the topmost vertice */

	var vertices = createArray(3);
	vertices[0] = window.ponto2d[triangle.a];
	vertices[1] = window.ponto2d[triangle.b];
	vertices[2] = window.ponto2d[triangle.c];
	vertices = Ponto2d.sortPoints(vertices);
	var v1 = vertices[0];
	var v2 = vertices[1];
	var v3 = vertices[2];

	/* here we know that v1.y <= v2.y <= v3.y */
	/* check for trivial case of bottom-flat triangle */
	if (v2.y == v3.y) {
	    fillBottomFlatTriangle(v1, v2, v3, v1.originalVertex, v2.originalVertex, v3.originalVertex);
	}
	/* check for trivial case of top-flat triangle */
	else if (v1.y == v2.y) {
	    fillTopFlatTriangle(v1, v2, v3, v1.originalVertex, v2.originalVertex, v3.originalVertex);
	} else {
	    // debugger;
	    //general case - split the triangle in a topflat and bottom-flat one
	    var v4 = new Ponto2d(
		Math.floor((v1.x + ((v2.y - v1.y) / (v3.y - v1.y)) * (v3.x - v1.x))), v2.y);
	    var lp = baricentro(v4, v1, v2, v3);
	    // if (lp.u < 0 || lp.v < 0 || lp.w < 0) {
	    // 	v4.x++;
	    // 	lp = baricentro(v4, v1, v2, v3);
	    // }
	    // if (lp.u < 0 || lp.v < 0 || lp.w < 0) {
	    // 	v4.x -= 2;
	    // 	lp = baricentro(v4, v1, v2, v3);
	    // }
	    // if (lp.u < 0 || lp.v < 0 || lp.w < 0) {
	    // 	v4.x++;
	    // 	v4.y++;
	    // 	lp = baricentro(v4, v1, v2, v3);
	    // }
	    // if (lp.u < 0 || lp.v < 0 || lp.w < 0) {
	    // 	v4.y-= 2;
	    // 	lp = baricentro(v4, v1, v2, v3);
	    // }
	    // if (lp.u < 0 || lp.v < 0 || lp.w < 0) {
	    // 	return; // foda-se
	    // }
	    var P = new Ponto(window.objeto.V[v1.originalVertex].x * lp.u +
			      window.objeto.V[v2.originalVertex].x * lp.v +
			      window.objeto.V[v3.originalVertex].x * lp.w,
			      window.objeto.V[v1.originalVertex].y * lp.u +
			      window.objeto.V[v2.originalVertex].y * lp.v +
			      window.objeto.V[v3.originalVertex].y * lp.w,
			      window.objeto.V[v1.originalVertex].z * lp.u +
			      window.objeto.V[v2.originalVertex].z * lp.v +
			      window.objeto.V[v3.originalVertex].z * lp.w);
	    var N = new Vetor(window.objeto.V[v1.originalVertex].N.x * lp.u +
			      window.objeto.V[v2.originalVertex].N.x * lp.v +
			      window.objeto.V[v3.originalVertex].N.x * lp.w,
			      window.objeto.V[v1.originalVertex].N.y * lp.u +
			      window.objeto.V[v2.originalVertex].N.y * lp.v +
			      window.objeto.V[v3.originalVertex].N.y * lp.w,
			      window.objeto.V[v1.originalVertex].N.z * lp.u +
			      window.objeto.V[v2.originalVertex].N.z * lp.v +
			      window.objeto.V[v3.originalVertex].N.z * lp.w);
	    N = N.normalizado();
	    fillBottomFlatTriangle(v1, v2, v4, v1.originalVertex, v2.originalVertex, null, P, N);
	    fillTopFlatTriangle(v2, v4, v3, v2.originalVertex, null, v3.originalVertex, P, N);
	}
    }

    function renderObj() {
	var faces = window.objeto.F;
	for (const face of faces) {
	    debugger;
	    drawTriangle(face);
	    var meio = new Ponto(1/3*window.objeto.V[face.a].x +
				  1/3*window.objeto.V[face.b].x +
				  1/3*window.objeto.V[face.c].x,
				  1/3*window.objeto.V[face.a].y +
				  1/3*window.objeto.V[face.b].y +
				  1/3*window.objeto.V[face.c].y,
				  1/3*window.objeto.V[face.a].z +
				  1/3*window.objeto.V[face.b].z +
				  1/3*window.objeto.V[face.c].z);				  
				  
	    // drawArrow(1/3*window.ponto2d[face.a].x +
	    // 	      1/3*window.ponto2d[face.b].x +
	    // 	      1/3*window.ponto2d[face.c].x,
	    // 	      1/3*window.ponto2d[face.a].y +
	    // 	      1/3*window.ponto2d[face.b].y +
	    // 	      1/3*window.ponto2d[face.c].y, meio, face.N, true);
	}
	for (var i = 0; i < window.ponto2d.length; i++) {
	    var v = window.ponto2d[i];
	    // drawArrow(v.x, v.y, window.objeto.V[i], window.objeto.V[i].N, true);
	}
    }

    function drawArrow(xf, y, P, N, important) {
	var N2 = P.mais(N);
	var N2d = Ponto2d.threeDPD(N2, 0, p.width, p.height);

	var strWght = 1;
	var lenghtner = 20;
	var r = 3;
	if (important) {
	    debugger;
	    strWght = 2;
	    lenghtner = 40;
	    r = 5;
	}
	p.strokeWeight(strWght);

	p.stroke(0, 0, 0);
	var vecNormaBaseada = {
	    x: (N2d.x-xf),
	    y: (N2d.y-y)
	};
	var norma = Math.sqrt(vecNormaBaseada.x*vecNormaBaseada.x +
			      vecNormaBaseada.y*vecNormaBaseada.y);
	vecNormaBaseada.x = lenghtner * vecNormaBaseada.x/norma;
	vecNormaBaseada.y = lenghtner * vecNormaBaseada.y/norma;
	
	var x1 = {
	    x: xf,
	    y: y
	};
	var x2 = {
	    x: xf + vecNormaBaseada.x,
	    y: y + vecNormaBaseada.y
	};	

	if (Math.abs(N2d.x) <= 0.01 && Math.abs(N2d.y) <= 0.01) {
	    if (N.z < 0) {
		// desenha um X pra indicar que entra na tela
		p.line(x1.x - 3, x1.y - 3, x1.x + 3, x1.x + 3);
		p.line(x1.x + 3, x1.y - 3, x1.x - 3, x1.y + 3);
	    } else {
		// desenha uma bolinha no meio pra indicar que sai da tela
		p.fill(0, 0, 0);
		p.ellipse(xf, y, 1, 1);
	    }
	} else {
	    p.ellipse(xf, y, r, r);
	    p.line(x1.x, x1.y, x2.x, x2.y);
	}

	var offset = r;
	var angle = Math.atan2(x1.y - x2.y, x1.x - x2.x); //gets the angle of the line
	p.translate(x2.x, x2.y); //translates to the destination vertex
	p.rotate(angle-Math.PI/2); //rotates the arrow point
	p.triangle(-offset*0.5, offset, offset*0.5, offset, 0, -offset/2); //draws the arrow point as a triangle
	p.rotate(-(angle-Math.PI/2)); //rotates back
	p.translate(-x2.x, -x2.y); //translates back
	p.strokeWeight(1);
    }
};
