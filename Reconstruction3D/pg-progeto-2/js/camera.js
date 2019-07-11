function Camera(values) {
    var proj = (values.V).projecao(values.N);
    this.C = null;
    this.N = null;
    this.V = null;
    this.U = null;
    this.d = null;
    this.h = null;
    this.V = (values.V).menos(proj);

    this.V = (this.V).normalizado();

    this.N = (values.N).normalizado();

    this.U = (this.N).produtoVetorial(this.V).normalizado();

    this.h = values.h;
    this.d = values.d;

    this.C = values.C;

//    this.I_e_alfa = 
}
