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
    ,<0.12249007092053685,0.08710502079859449,0.0>,0.05
    ,<0.10626159350336704,0.09879258624624064,0.0>,0.05
    ,<0.09002482764333548,0.11046547391281675,0.0>,0.05
    ,<0.0737934623966358,0.12214262876933696,0.0>,0.05
    ,<0.0575886850008036,0.13385317900848015,0.0>,0.05
    ,<0.04143872057947053,0.1456353142456032,0.0>,0.05
    ,<0.025378530301012715,0.15753502360868088,0.0>,0.05
    ,<0.009449675034002392,0.1696046721943658,0.0>,0.05
    ,<-0.006299671694062766,0.18190140311868727,0.0>,0.05
    ,<-0.021814598665561548,0.19448535900845892,0.0>,0.05
    ,<-0.0370332779413029,0.20741772482654153,0.0>,0.05
    ,<-0.05188680674668537,0.22075860285851398,0.0>,0.05
    ,<-0.06629915925688157,0.2345647438855972,0.0>,0.05
    ,<-0.08018734444779184,0.2488871748276005,0.0>,0.05
    ,<-0.09346186689958677,0.2637687843596842,0.0>,0.05
    ,<-0.1060275778217143,0.27924195119704326,0.0>,0.05
    ,<-0.11778498396998952,0.295326324274567,0.0>,0.05
    ,<-0.1286320504267488,0.31202688478358404,0.0>,0.05
    ,<-0.13846649146444703,0.32933243400204276,0.0>,0.05
    ,<-0.1471884935936955,0.3472146523063833,0.0>,0.05
    ,<-0.15470376167044417,0.3656278607265219,0.0>,0.05
    ,<-0.16092672860800958,0.3845095841502169,0.0>,0.05
    ,<-0.16578372924554244,0.4037819655149784,0.0>,0.05
    ,<-0.169215916514464,0.42335401680221807,0.0>,0.05
    ,<-0.1711816984018028,0.44312462149320914,0.0>,0.05
    ,<-0.17165850016974984,0.4629861341648445,0.0>,0.05
    ,<-0.17064370602000015,0.4828283644562116,0.0>,0.05
    ,<-0.16815470281081507,0.502542694912504,0.0>,0.05
    ,<-0.16422802659131464,0.522026069233016,0.0>,0.05
    ,<-0.15891769030741182,0.5411846030234202,0.0>,0.05
    ,<-0.15229283782909714,0.5599366088076292,0.0>,0.05
    ,<-0.14443491705773057,0.5782148863980621,0.0>,0.05
    ,<-0.13543458878848086,0.5959681982673485,0.0>,0.05
    ,<-0.12538858703776548,0.6131619192883829,0.0>,0.05
    ,<-0.11439672398049594,0.6297779115107188,0.0>,0.05
    ,<-0.10255919376392786,0.6458137221524812,0.0>,0.05
    ,<-0.0899742814812986,0.6612812332121523,0.0>,0.05
    ,<-0.07673653344062585,0.6762049038813045,0.0>,0.05
    ,<-0.06293539822880401,0.6906197449239727,0.0>,0.05
    ,<-0.048654309528168106,0.7045691504117563,0.0>,0.05
    ,<-0.03397015264554316,0.7181026920368296,0.0>,0.05
    ,<-0.018953038804974676,0.731273957444018,0.0>,0.05
    ,<-0.003666302328063281,0.7441384914421768,0.0>,0.05
    ,<0.01183336403036181,0.7567518781091194,0.0>,0.05
    ,<0.02749571481254458,0.7691679856968946,0.0>,0.05
    ,<0.04327676668730735,0.7814373831410207,0.0>,0.05
    ,<0.059138677416846336,0.7936059284337545,0.0>,0.05
    ,<0.07504972796144578,0.8057135220861629,0.0>,0.05
    ,<0.09098443119951863,0.8177930143811748,0.0>,0.05
    ,<0.10692377363808965,0.8298692497481366,0.0>,0.05
    ,<0.12285558104867948,0.8419582265985086,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }