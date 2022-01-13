#include "../default.inc"

camera{
    location <15.0,10.5,-15.0>
    angle 30
    look_at <4.0,2.7,2.0>
    sky <0,1,0>
    right x*image_width/image_height
}
light_source{
    <15.0,10.5,-15.0>
    color rgb<0.09,0.09,0.1>
}
light_source{
    <1500,2500,-1000>
    color White
}

sphere_sweep {
    linear_spline 51
    ,<-0.06760641784565484,0.4702254083109074,0.0>,0.05
    ,<-0.061798997129492966,0.45108956201495815,0.0>,0.05
    ,<-0.056229947118272124,0.4318877061649894,0.0>,0.05
    ,<-0.05129965774993792,0.4125163414333848,0.0>,0.05
    ,<-0.04753404424360696,0.3928890901718563,0.0>,0.05
    ,<-0.04554168049388246,0.37300667192954623,0.0>,0.05
    ,<-0.04596149882471714,0.3530319322917815,0.0>,0.05
    ,<-0.04938904907342599,0.33335168050267355,0.0>,0.05
    ,<-0.056278155442398684,0.31460348107706165,0.0>,0.05
    ,<-0.06682976556786902,0.29764735846005336,0.0>,0.05
    ,<-0.0808950678577546,0.2834731837204024,0.0>,0.05
    ,<-0.09792715161327119,0.2730541878474002,0.0>,0.05
    ,<-0.1170077781077719,0.267179016371682,0.0>,0.05
    ,<-0.13695332488808454,0.26630796015243396,0.0>,0.05
    ,<-0.15647575662260022,0.2704945929564356,0.0>,0.05
    ,<-0.17435433502185982,0.27939205573881526,0.0>,0.05
    ,<-0.18957206088808992,0.29233314655304327,0.0>,0.05
    ,<-0.20139197453499952,0.30844722501023825,0.0>,0.05
    ,<-0.2093751484656621,0.32677839686605653,0.0>,0.05
    ,<-0.21335562840952363,0.3463833905834767,0.0>,0.05
    ,<-0.21339134347998273,0.36639999192422196,0.0>,0.05
    ,<-0.2097075261933511,0.3860867988659716,0.0>,0.05
    ,<-0.20264374088775158,0.4048407748361215,0.0>,0.05
    ,<-0.19260980724096308,0.4222009063645617,0.0>,0.05
    ,<-0.18005032505781976,0.4378445493299549,0.0>,0.05
    ,<-0.16541599747096156,0.4515801072123472,0.0>,0.05
    ,<-0.1491406869165112,0.46333832325547875,0.0>,0.05
    ,<-0.13162327230836743,0.4731636170893105,0.0>,0.05
    ,<-0.11321388470167175,0.48120635798156447,0.0>,0.05
    ,<-0.09420492023938797,0.48771675823053595,0.0>,0.05
    ,<-0.07482827501967648,0.49304094650083635,0.0>,0.05
    ,<-0.055260697380845665,0.497615654144236,0.0>,0.05
    ,<-0.03564111815096033,0.5019570691408571,0.0>,0.05
    ,<-0.016103585903743382,0.5066428116878186,0.0>,0.05
    ,<0.0031753364175440904,0.5122838038609482,0.0>,0.05
    ,<0.021922605511508545,0.5194799368589473,0.0>,0.05
    ,<0.039724407489119797,0.5287534526732937,0.0>,0.05
    ,<0.05601555150291333,0.5404610502657203,0.0>,0.05
    ,<0.07012313651190126,0.5547065465867048,0.0>,0.05
    ,<0.08136439239516766,0.5712906809577969,0.0>,0.05
    ,<0.08917590714931702,0.589724829475683,0.0>,0.05
    ,<0.09323285561622216,0.6093172709569346,0.0>,0.05
    ,<0.0935161455313515,0.6293126957702442,0.0>,0.05
    ,<0.0903057472157095,0.6490429180256567,0.0>,0.05
    ,<0.08410910128234284,0.6680430355551675,0.0>,0.05
    ,<0.07555690657193122,0.6861038006110743,0.0>,0.05
    ,<0.06530299138252416,0.7032563689909859,0.0>,0.05
    ,<0.05395121342992298,0.7197057541072971,0.0>,0.05
    ,<0.042010816390651615,0.7357372753275865,0.0>,0.05
    ,<0.029864278138668397,0.7516182885545158,0.0>,0.05
    ,<0.01772603706262348,0.7675109959036326,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
