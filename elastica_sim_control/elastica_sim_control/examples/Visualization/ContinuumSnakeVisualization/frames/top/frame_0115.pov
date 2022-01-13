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
    ,<-0.07447574113066742,0.47409378100533717,0.0>,0.05
    ,<-0.06705292968933556,0.4555242837372461,0.0>,0.05
    ,<-0.05986118213793128,0.43686803399272395,0.0>,0.05
    ,<-0.05329170482018558,0.41798728964595266,0.0>,0.05
    ,<-0.04786278093025316,0.39875068898514643,0.0>,0.05
    ,<-0.044183042005454795,0.3791070186434309,0.0>,0.05
    ,<-0.04290567621139687,0.3591649768338807,0.0>,0.05
    ,<-0.04465989001028675,0.33926170573006914,0.0>,0.05
    ,<-0.049954456511350574,0.3199984117120268,0.0>,0.05
    ,<-0.05906335828829137,0.30222180836004475,0.0>,0.05
    ,<-0.07191991008836703,0.2869395464455377,0.0>,0.05
    ,<-0.08805485707293396,0.27517692637140256,0.0>,0.05
    ,<-0.10660826571028106,0.26780507103075507,0.0>,0.05
    ,<-0.12642357559924414,0.2653862172660122,0.0>,0.05
    ,<-0.14620328196230742,0.2680800082752776,0.0>,0.05
    ,<-0.16468329966888826,0.27563434053505703,0.0>,0.05
    ,<-0.18077851739669482,0.2874540679893716,0.0>,0.05
    ,<-0.19367101109396684,0.3027125986695617,0.0>,0.05
    ,<-0.2028393745968216,0.32047040373305846,0.0>,0.05
    ,<-0.20804241927632508,0.33977715669451747,0.0>,0.05
    ,<-0.2092755193452036,0.3597463754677919,0.0>,0.05
    ,<-0.20671637532077297,0.3796016531235315,0.0>,0.05
    ,<-0.2006720578251507,0.3986998748319831,0.0>,0.05
    ,<-0.19153351250134376,0.4165392467974132,0.0>,0.05
    ,<-0.1797379577336512,0.4327587346395291,0.0>,0.05
    ,<-0.16573784071018788,0.4471327361508834,0.0>,0.05
    ,<-0.14997559438588726,0.4595634581648465,0.0>,0.05
    ,<-0.13286345195901697,0.47007263345440414,0.0>,0.05
    ,<-0.11476799357975871,0.47879365595533,0.0>,0.05
    ,<-0.0959998606366011,0.48596496349608304,0.0>,0.05
    ,<-0.07681007626399611,0.4919253250327256,0.0>,0.05
    ,<-0.05739470849220265,0.4971075392580283,0.0>,0.05
    ,<-0.0379115866344148,0.5020260956166234,0.0>,0.05
    ,<-0.018512679905996466,0.5072577500466741,0.0>,0.05
    ,<0.0006090932434159014,0.5134119014831713,0.0>,0.05
    ,<0.019166673060621944,0.521084900366975,0.0>,0.05
    ,<0.03673566549462268,0.5307925487018583,0.0>,0.05
    ,<0.05274511799723144,0.5428822278978975,0.0>,0.05
    ,<0.0665223731698789,0.5574467473082599,0.0>,0.05
    ,<0.07739120745388064,0.5742762954922467,0.0>,0.05
    ,<0.0848000937351275,0.5928746677756764,0.0>,0.05
    ,<0.08843921434088489,0.6125475733627703,0.0>,0.05
    ,<0.08830465299353718,0.6325429965989794,0.0>,0.05
    ,<0.08468887558039728,0.6522013302880328,0.0>,0.05
    ,<0.07810718737854534,0.6710698573474342,0.0>,0.05
    ,<0.06919287857282648,0.6889530347474491,0.0>,0.05
    ,<0.05859769993365915,0.7058954225918646,0.0>,0.05
    ,<0.04692023689657376,0.7221139408601238,0.0>,0.05
    ,<0.034663153096956376,0.737903684780439,0.0>,0.05
    ,<0.022202960963318225,0.7535392205439743,0.0>,0.05
    ,<0.009750454904834564,0.7691867105690996,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
