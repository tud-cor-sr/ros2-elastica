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
    ,<-0.19428613234889114,0.47009362023859697,0.0>,0.05
    ,<-0.19090667024379077,0.45038851038904476,0.0>,0.05
    ,<-0.187167820457958,0.43076271525590487,0.0>,0.05
    ,<-0.18250744782992748,0.4113483747906332,0.0>,0.05
    ,<-0.1762706876073028,0.39239288154961727,0.0>,0.05
    ,<-0.16783791973720874,0.3743151319969099,0.0>,0.05
    ,<-0.1567584578086512,0.35773006753756365,0.0>,0.05
    ,<-0.14287387925879888,0.3434056172592504,0.0>,0.05
    ,<-0.12639492221084725,0.33214254085712075,0.0>,0.05
    ,<-0.10789240476472121,0.32460595952462584,0.0>,0.05
    ,<-0.0881871858071796,0.32116854644403525,0.0>,0.05
    ,<-0.06816750727280886,0.32182785896555033,0.0>,0.05
    ,<-0.0485972345138044,0.32622820730317664,0.0>,0.05
    ,<-0.029981619790079718,0.3337695200547353,0.0>,0.05
    ,<-0.012527485723567813,0.34375256032114354,0.0>,0.05
    ,<0.0038058454800566203,0.35550985106559574,0.0>,0.05
    ,<0.019196943253454536,0.36849625710476624,0.0>,0.05
    ,<0.03384572559217456,0.3823282543646561,0.0>,0.05
    ,<0.0478861383085132,0.39678756736188125,0.0>,0.05
    ,<0.0613349210753005,0.4118079304616557,0.0>,0.05
    ,<0.07406991413135001,0.42744835926176494,0.0>,0.05
    ,<0.08582943356829965,0.44384747625131604,0.0>,0.05
    ,<0.09623127207597783,0.4611546699188515,0.0>,0.05
    ,<0.10482056039097544,0.47944551871186974,0.0>,0.05
    ,<0.11114066626718858,0.49865449025286734,0.0>,0.05
    ,<0.11481150792309347,0.5185534126008737,0.0>,0.05
    ,<0.1155997810358748,0.5387814862170189,0.0>,0.05
    ,<0.1134583721496161,0.5589150959772317,0.0>,0.05
    ,<0.10851881101642923,0.5785491786941623,0.0>,0.05
    ,<0.1010376586720887,0.597356313775548,0.0>,0.05
    ,<0.0913141325158202,0.6150985073281146,0.0>,0.05
    ,<0.07961663947849472,0.6315930349856146,0.0>,0.05
    ,<0.06614722765190216,0.6466585249754955,0.0>,0.05
    ,<0.051042687959954966,0.6600620921081746,0.0>,0.05
    ,<0.03440776193863544,0.6714805297100421,0.0>,0.05
    ,<0.016375537069729447,0.6804859871869244,0.0>,0.05
    ,<-0.0028145173762378294,0.6865684059974567,0.0>,0.05
    ,<-0.022742473108375166,0.689207537062116,0.0>,0.05
    ,<-0.04277533689109946,0.6879873037359926,0.0>,0.05
    ,<-0.0621060273037328,0.6827141113606295,0.0>,0.05
    ,<-0.07986320418960065,0.6734995952149653,0.0>,0.05
    ,<-0.09526322741878047,0.6607726325730418,0.0>,0.05
    ,<-0.107755516122946,0.645208559931623,0.0>,0.05
    ,<-0.1171137120380135,0.6275968055521804,0.0>,0.05
    ,<-0.12344844469920552,0.6086927446178481,0.0>,0.05
    ,<-0.12714937816663566,0.5891019960885404,0.0>,0.05
    ,<-0.12878624688341195,0.5692266528328354,0.0>,0.05
    ,<-0.12900088403943344,0.5492757514543545,0.0>,0.05
    ,<-0.12840845077669538,0.5293202428063645,0.0>,0.05
    ,<-0.12750830074542266,0.5093623333665837,0.0>,0.05
    ,<-0.12659449138140141,0.4893905206674302,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
