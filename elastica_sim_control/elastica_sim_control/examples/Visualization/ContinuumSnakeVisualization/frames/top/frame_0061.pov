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
    ,<-0.28672715179902186,0.5216402547349486,0.0>,0.05
    ,<-0.267974240017295,0.5147800894031545,0.0>,0.05
    ,<-0.2494163894409046,0.5075343730598446,0.0>,0.05
    ,<-0.23109773190697777,0.4997989363345664,0.0>,0.05
    ,<-0.2130631120525856,0.4914843076908263,0.0>,0.05
    ,<-0.19535049329657686,0.4825386545933174,0.0>,0.05
    ,<-0.1779793240581124,0.4729631116471153,0.0>,0.05
    ,<-0.16093917304099734,0.46282137421737896,0.0>,0.05
    ,<-0.14417778095451494,0.4522426940544628,0.0>,0.05
    ,<-0.12759480208081128,0.4414214548112192,0.0>,0.05
    ,<-0.11104276572738155,0.43061425618374266,0.0>,0.05
    ,<-0.09433945679634381,0.42013732366130824,0.0>,0.05
    ,<-0.07729272171612422,0.4103643009542728,0.0>,0.05
    ,<-0.059736458283408914,0.4017218711063387,0.0>,0.05
    ,<-0.041575523436650975,0.39467884845032536,0.0>,0.05
    ,<-0.022830108656919038,0.38972124167666483,0.0>,0.05
    ,<-0.003670568067482189,0.38731008752151164,0.0>,0.05
    ,<0.015572799842638855,0.38782796703471445,0.0>,0.05
    ,<0.03442974298914728,0.3915254685940283,0.0>,0.05
    ,<0.05233826127783738,0.39847989636901376,0.0>,0.05
    ,<0.06871214114070343,0.4085768922372568,0.0>,0.05
    ,<0.08301172161397061,0.4215209708353269,0.0>,0.05
    ,<0.09480399189994985,0.43687384703552273,0.0>,0.05
    ,<0.10379778465118776,0.45411006223271194,0.0>,0.05
    ,<0.10985052903321524,0.472676682665311,0.0>,0.05
    ,<0.11295057070460528,0.4920431619390084,0.0>,0.05
    ,<0.11318261838114908,0.5117328245739302,0.0>,0.05
    ,<0.1106877511295195,0.5313353905797774,0.0>,0.05
    ,<0.10562548033755392,0.5504990325681159,0.0>,0.05
    ,<0.09814420970104869,0.5689107651715813,0.0>,0.05
    ,<0.08836381878672298,0.5862641786747051,0.0>,0.05
    ,<0.07637508414699459,0.6022254154692868,0.0>,0.05
    ,<0.06226246918391809,0.6164004858108283,0.0>,0.05
    ,<0.0461468576157633,0.6283161789554492,0.0>,0.05
    ,<0.02824715772694822,0.63742071577873,0.0>,0.05
    ,<0.008948972422802659,0.6431178712995947,0.0>,0.05
    ,<-0.011135955750606118,0.6448428448615774,0.0>,0.05
    ,<-0.031151403765598565,0.6421807955904157,0.0>,0.05
    ,<-0.05005018359491391,0.6349997841452041,0.0>,0.05
    ,<-0.06672809549354013,0.6235447383604377,0.0>,0.05
    ,<-0.080206594673265,0.6084479789692945,0.0>,0.05
    ,<-0.08980782767898979,0.5906376031575761,0.0>,0.05
    ,<-0.09526216596958893,0.5711664074381027,0.0>,0.05
    ,<-0.09671643665085547,0.5510164205344265,0.0>,0.05
    ,<-0.09465181052921938,0.5309417150995557,0.0>,0.05
    ,<-0.0897529420354339,0.5113892366722402,0.0>,0.05
    ,<-0.08277615828290771,0.4925053436424989,0.0>,0.05
    ,<-0.07444778040102126,0.47420686380694704,0.0>,0.05
    ,<-0.06539727716064184,0.4562867860413573,0.0>,0.05
    ,<-0.056107956196743136,0.43852329567156817,0.0>,0.05
    ,<-0.04686200115761375,0.42077199131395004,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
