#include "../default.inc"

camera{
    location <0,15,3>
    angle 30
    look_at <0.0,0,3>
    sky <-1,0,0>
    right x*image_width/image_height
}
light_source{
    <0.0,8.0,5.0>
    color rgb<0.09,0.09,0.1>
}
light_source{
    <1500,2500,-1000>
    color White
}

sphere_sweep {
    linear_spline 51
    ,<0.150319192756126,0.2613353267006744,0.0>,0.05
    ,<0.13047802306648348,0.2588270377036425,0.0>,0.05
    ,<0.11064402396066836,0.25627634599044075,0.0>,0.05
    ,<0.09080968233189304,0.2537446931709913,0.0>,0.05
    ,<0.0709643625456986,0.25132541029750666,0.0>,0.05
    ,<0.051096628145813096,0.24914019194435935,0.0>,0.05
    ,<0.03119786781987393,0.24733551911448765,0.0>,0.05
    ,<0.011267251555059905,0.2460788430816179,0.0>,0.05
    ,<-0.00868216641831441,0.24555423148302977,0.0>,0.05
    ,<-0.028616569227918737,0.2459570918355934,0.0>,0.05
    ,<-0.04847432232143834,0.2474875632124361,0.0>,0.05
    ,<-0.06815954249220368,0.2503422434454011,0.0>,0.05
    ,<-0.08753728321026014,0.25470412997386543,0.0>,0.05
    ,<-0.10643141213635993,0.2607309990493047,0.0>,0.05
    ,<-0.12462619356283937,0.2685429030869436,0.0>,0.05
    ,<-0.14187230159926226,0.27820994221761025,0.0>,0.05
    ,<-0.1578974632864085,0.2897418444250845,0.0>,0.05
    ,<-0.17242123034379386,0.30308103618527216,0.0>,0.05
    ,<-0.18517259854722273,0.3181006885623584,0.0>,0.05
    ,<-0.1959085575894797,0.3346086809828233,0.0>,0.05
    ,<-0.20443128762029644,0.3523575778544868,0.0>,0.05
    ,<-0.21060179346488145,0.3710597796961911,0.0>,0.05
    ,<-0.21434825607256808,0.39040615348624574,0.0>,0.05
    ,<-0.2156682121709406,0.4100859150885695,0.0>,0.05
    ,<-0.21462465078664655,0.4298054199105619,0.0>,0.05
    ,<-0.21133700995732527,0.44930382838395605,0.0>,0.05
    ,<-0.20596869777264196,0.4683642881746694,0.0>,0.05
    ,<-0.19871302312464056,0.4868200532600611,0.0>,0.05
    ,<-0.18977931139198562,0.5045557799745372,0.0>,0.05
    ,<-0.17938060125212107,0.5215047784846271,0.0>,0.05
    ,<-0.16772377282906634,0.5376433762181007,0.0>,0.05
    ,<-0.15500244863622534,0.5529835627686881,0.0>,0.05
    ,<-0.14139255134998177,0.567565001752975,0.0>,0.05
    ,<-0.12705014529152503,0.581447211725595,0.0>,0.05
    ,<-0.11211102789803973,0.5947024771967508,0.0>,0.05
    ,<-0.09669153978129809,0.6074097811411836,0.0>,0.05
    ,<-0.08089010894928284,0.6196498712179296,0.0>,0.05
    ,<-0.06478914737128993,0.6315014381143369,0.0>,0.05
    ,<-0.04845703197250553,0.6430383029285955,0.0>,0.05
    ,<-0.03194998481513226,0.6543274888665152,0.0>,0.05
    ,<-0.015313775579888294,0.6654280230734868,0.0>,0.05
    ,<0.0014148085275451103,0.6763903494574012,0.0>,0.05
    ,<0.018206691269348808,0.6872562132822932,0.0>,0.05
    ,<0.03503943518327531,0.6980589348366752,0.0>,0.05
    ,<0.05189629096993974,0.7088239599475415,0.0>,0.05
    ,<0.06876536652402691,0.719569639991558,0.0>,0.05
    ,<0.08563883399980339,0.7303081518834178,0.0>,0.05
    ,<0.10251219371873141,0.7410465389275254,0.0>,0.05
    ,<0.1193835253111066,0.751787800740201,0.0>,0.05
    ,<0.13625276907107473,0.7625320354810841,0.0>,0.05
    ,<0.1531209801388069,0.7732775769828579,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }