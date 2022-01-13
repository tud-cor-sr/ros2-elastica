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
    ,<0.14546770122715127,0.14759521716591173,0.0>,0.05
    ,<0.12684120902266907,0.15487634524598756,0.0>,0.05
    ,<0.10820807595025884,0.16213525210537885,0.0>,0.05
    ,<0.08958021696891749,0.16940222429114526,0.0>,0.05
    ,<0.07097611393524837,0.17672364494414491,0.0>,0.05
    ,<0.05242068214045517,0.18416017678121502,0.0>,0.05
    ,<0.03394548748862403,0.1917847621645013,0.0>,0.05
    ,<0.015589326428035209,0.19968040175933166,0.0>,0.05
    ,<-0.0026008703110241647,0.2079376756513849,0.0>,0.05
    ,<-0.020568896277651263,0.21665197173243675,0.0>,0.05
    ,<-0.03824802222461994,0.22592039176459802,0.0>,0.05
    ,<-0.055559814923675954,0.23583831936147925,0.0>,0.05
    ,<-0.07241322563488639,0.2464956612762538,0.0>,0.05
    ,<-0.08870417054323494,0.25797281542709644,0.0>,0.05
    ,<-0.10431582554565758,0.2703364758169169,0.0>,0.05
    ,<-0.11911983293730875,0.2836354514386607,0.0>,0.05
    ,<-0.13297856256074328,0.2978967444130806,0.0>,0.05
    ,<-0.14574848417562322,0.3131221902056966,0.0>,0.05
    ,<-0.15728459363568706,0.32928599419290616,0.0>,0.05
    ,<-0.1674457049661806,0.3463334915537422,0.0>,0.05
    ,<-0.176100289040207,0.3641814004795658,0.0>,0.05
    ,<-0.1831324302611717,0.3827197321260817,0.0>,0.05
    ,<-0.18844740595004592,0.40181537168523346,0.0>,0.05
    ,<-0.19197638719265073,0.4213171737456435,0.0>,0.05
    ,<-0.19367982137832263,0.441062246523722,0.0>,0.05
    ,<-0.19354918124285037,0.46088296293470093,0.0>,0.05
    ,<-0.19160693497271258,0.480614154626532,0.0>,0.05
    ,<-0.1879047806582778,0.5000999334309651,0.0>,0.05
    ,<-0.18252036567793165,0.5191996425462767,0.0>,0.05
    ,<-0.175552850627678,0.5377925573548058,0.0>,0.05
    ,<-0.16711775992889874,0.5557811080270716,0.0>,0.05
    ,<-0.15734158019707847,0.573092560564166,0.0>,0.05
    ,<-0.146356529296952,0.5896792396761936,0.0>,0.05
    ,<-0.13429583628664155,0.6055174931035172,0.0>,0.05
    ,<-0.12128976705266288,0.6206056663823234,0.0>,0.05
    ,<-0.10746251802961293,0.6349613859904266,0.0>,0.05
    ,<-0.09293000094923538,0.6486184367426209,0.0>,0.05
    ,<-0.07779846036468141,0.6616234838598793,0.0>,0.05
    ,<-0.062163812185988376,0.6740328367236695,0.0>,0.05
    ,<-0.04611155997719621,0.6859093962845848,0.0>,0.05
    ,<-0.029717137449688794,0.6973198744073054,0.0>,0.05
    ,<-0.013046530871708435,0.708332329986971,0.0>,0.05
    ,<0.0038429478374191126,0.7190140324562285,0.0>,0.05
    ,<0.020901845063148756,0.7294296404418745,0.0>,0.05
    ,<0.03808779029594421,0.7396396691799917,0.0>,0.05
    ,<0.05536482204148003,0.749699213256864,0.0>,0.05
    ,<0.07270285405500754,0.7596568894209632,0.0>,0.05
    ,<0.09007730139285385,0.7695539645946472,0.0>,0.05
    ,<0.10746887028767285,0.7794236367117833,0.0>,0.05
    ,<0.12486350066912946,0.7892904372649286,0.0>,0.05
    ,<0.14225243942958124,0.7991697261974969,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
