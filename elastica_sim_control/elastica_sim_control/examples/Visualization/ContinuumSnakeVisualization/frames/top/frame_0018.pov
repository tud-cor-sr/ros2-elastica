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
    ,<-0.2126998905224867,0.28156872335003436,0.0>,0.05
    ,<-0.19661067703794877,0.26970409263907175,0.0>,0.05
    ,<-0.18030907605369556,0.2581631820302263,0.0>,0.05
    ,<-0.1634950839249749,0.24740868550369158,0.0>,0.05
    ,<-0.14588313215309837,0.2380347180544132,0.0>,0.05
    ,<-0.1273211802972623,0.23071686047710244,0.0>,0.05
    ,<-0.10788790449309613,0.22614020360075257,0.0>,0.05
    ,<-0.08793669176654162,0.22489485455420105,0.0>,0.05
    ,<-0.06806154282349273,0.22735676725783763,0.0>,0.05
    ,<-0.048982435245157586,0.23359510147579196,0.0>,0.05
    ,<-0.03138048689482758,0.2433486409747328,0.0>,0.05
    ,<-0.01573828126853401,0.25608785818369767,0.0>,0.05
    ,<-0.002240012024296654,0.2711397890484616,0.0>,0.05
    ,<0.009241590892363816,0.2878253258193465,0.0>,0.05
    ,<0.019080577083416664,0.30556028279831765,0.0>,0.05
    ,<0.027785833049643303,0.32389877787767385,0.0>,0.05
    ,<0.03587389008149666,0.3425298763197377,0.0>,0.05
    ,<0.04376605468673025,0.3612510467523723,0.0>,0.05
    ,<0.05172697568028933,0.3799457702945412,0.0>,0.05
    ,<0.05984043528672027,0.39857635494493254,0.0>,0.05
    ,<0.06801490832450663,0.4171830700381295,0.0>,0.05
    ,<0.0760108218119268,0.43587319403158636,0.0>,0.05
    ,<0.0834842535548032,0.4547881936759734,0.0>,0.05
    ,<0.09004805896155955,0.4740507373628436,0.0>,0.05
    ,<0.09533621870409738,0.4937158174494286,0.0>,0.05
    ,<0.09905851051615805,0.513749052705364,0.0>,0.05
    ,<0.10104243625960786,0.5340376301612878,0.0>,0.05
    ,<0.10125329393169587,0.5544267849529632,0.0>,0.05
    ,<0.09978421621960824,0.5747646151033047,0.0>,0.05
    ,<0.09681481473934224,0.5949351898795235,0.0>,0.05
    ,<0.09254417656176041,0.6148656356128321,0.0>,0.05
    ,<0.08712333400213586,0.6345075673317526,0.0>,0.05
    ,<0.08061180096353701,0.653806815437731,0.0>,0.05
    ,<0.07295841888555384,0.6726712973720608,0.0>,0.05
    ,<0.06400647069907396,0.6909400304241756,0.0>,0.05
    ,<0.05352525472089081,0.7083543745620466,0.0>,0.05
    ,<0.04127058275539946,0.7245373317826342,0.0>,0.05
    ,<0.02707141622730877,0.7389964335498429,0.0>,0.05
    ,<0.010919725720043214,0.7511654682878852,0.0>,0.05
    ,<-0.0069731357750469816,0.7604829015919503,0.0>,0.05
    ,<-0.026174734992049566,0.7664936934779868,0.0>,0.05
    ,<-0.04609239288924243,0.7689455508306242,0.0>,0.05
    ,<-0.06608305795576376,0.7678462976490535,0.0>,0.05
    ,<-0.08557903334534327,0.7634620611040379,0.0>,0.05
    ,<-0.10419040610370713,0.7562598809612702,0.0>,0.05
    ,<-0.12175603544268572,0.7468192282283599,0.0>,0.05
    ,<-0.13833533145702762,0.7357434755835588,0.0>,0.05
    ,<-0.15415129878603964,0.7235929340872154,0.0>,0.05
    ,<-0.16950429753745846,0.7108428050681712,0.0>,0.05
    ,<-0.1846761905145251,0.69785250051023,0.0>,0.05
    ,<-0.19984002731824277,0.6848255555319152,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }