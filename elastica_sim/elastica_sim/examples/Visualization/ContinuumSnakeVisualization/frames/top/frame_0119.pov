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
    ,<-0.08246471532137642,0.47722849673209683,0.0>,0.05
    ,<-0.07318368607425745,0.4595139371723195,0.0>,0.05
    ,<-0.06412273947292985,0.4416888948004434,0.0>,0.05
    ,<-0.05565815121155476,0.4235758491920172,0.0>,0.05
    ,<-0.04829384744873867,0.40499065122863626,0.0>,0.05
    ,<-0.04263243888371722,0.38582006817663933,0.0>,0.05
    ,<-0.03933776631443674,0.36610639047418275,0.0>,0.05
    ,<-0.039073407513565285,0.3461232993807203,0.0>,0.05
    ,<-0.0424096138039551,0.3264217847380284,0.0>,0.05
    ,<-0.04970639409563557,0.3078236334795423,0.0>,0.05
    ,<-0.06099795680261216,0.29134765208278435,0.0>,0.05
    ,<-0.0759150273387242,0.2780720067332417,0.0>,0.05
    ,<-0.09367841970207674,0.2689597491013889,0.0>,0.05
    ,<-0.11317736789008168,0.26469261786804993,0.0>,0.05
    ,<-0.1331167724080059,0.2655596605630385,0.0>,0.05
    ,<-0.15219261347305948,0.27142919681081545,0.0>,0.05
    ,<-0.1692468435028656,0.28180258842624234,0.0>,0.05
    ,<-0.1833691437483952,0.2959177726412089,0.0>,0.05
    ,<-0.19393991945721564,0.31286651915736974,0.0>,0.05
    ,<-0.20062515762083824,0.3317004042325593,0.0>,0.05
    ,<-0.20334024512393126,0.3515121125107624,0.0>,0.05
    ,<-0.20219954475961024,0.3714891209890555,0.0>,0.05
    ,<-0.19746433875781214,0.39094375693183625,0.0>,0.05
    ,<-0.1894963170251174,0.4093267435838734,0.0>,0.05
    ,<-0.17871790757403216,0.4262307357115908,0.0>,0.05
    ,<-0.16557870074207712,0.4413878117311751,0.0>,0.05
    ,<-0.15052761336439818,0.4546635652243531,0.0>,0.05
    ,<-0.133990313508899,0.4660496469937935,0.0>,0.05
    ,<-0.1163517326823612,0.47565604275326956,0.0>,0.05
    ,<-0.09794417396217923,0.4837040809700321,0.0>,0.05
    ,<-0.0790424696187806,0.49052093560263177,0.0>,0.05
    ,<-0.05986772818792269,0.496532216523217,0.0>,0.05
    ,<-0.04060317870442721,0.5022481777071818,0.0>,0.05
    ,<-0.021425690367484897,0.5082424677999564,0.0>,0.05
    ,<-0.0025516138507152212,0.5151204511260317,0.0>,0.05
    ,<0.015711322518768233,0.5234715577591558,0.0>,0.05
    ,<0.03292263798029209,0.5338004464887685,0.0>,0.05
    ,<0.04850215375363199,0.5464390934002793,0.0>,0.05
    ,<0.0617768635554279,0.5614623489920143,0.0>,0.05
    ,<0.07207975777581044,0.5786430945392744,0.0>,0.05
    ,<0.07887681857021361,0.5974723120788819,0.0>,0.05
    ,<0.08188077591490638,0.6172505917744006,0.0>,0.05
    ,<0.08111080645645481,0.6372299133713769,0.0>,0.05
    ,<0.07687858503640282,0.6567629892993124,0.0>,0.05
    ,<0.06971163676152536,0.6754152117216333,0.0>,0.05
    ,<0.060247385882570975,0.6930117110443357,0.0>,0.05
    ,<0.04913448958164474,0.7096174295960161,0.0>,0.05
    ,<0.03696345750860241,0.7254674735624685,0.0>,0.05
    ,<0.02422681613206696,0.7408719082019902,0.0>,0.05
    ,<0.011291877635824432,0.7561163628340166,0.0>,0.05
    ,<-0.0016361504439774788,0.7713731037776712,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }