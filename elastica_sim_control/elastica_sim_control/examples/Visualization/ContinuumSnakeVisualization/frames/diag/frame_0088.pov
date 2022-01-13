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
    ,<-0.020325641606806027,0.3438891465094351,0.0>,0.05
    ,<-0.034991342980835165,0.3303030787717299,0.0>,0.05
    ,<-0.04981983907097664,0.31691905313909596,0.0>,0.05
    ,<-0.06506521086553,0.30403487592977446,0.0>,0.05
    ,<-0.08101774873304822,0.2920581562190813,0.0>,0.05
    ,<-0.09792911137982808,0.28149580471667945,0.0>,0.05
    ,<-0.11593230102729254,0.27293809212673564,0.0>,0.05
    ,<-0.13496707128237742,0.2670218125295294,0.0>,0.05
    ,<-0.15472652130735653,0.2643636516866236,0.0>,0.05
    ,<-0.1746430625635648,0.26546736706776614,0.0>,0.05
    ,<-0.1939275414403082,0.27062296678205827,0.0>,0.05
    ,<-0.2116623826396195,0.27982634067129225,0.0>,0.05
    ,<-0.22693163731233476,0.292747432878752,0.0>,0.05
    ,<-0.23895578638554066,0.3087619055649225,0.0>,0.05
    ,<-0.2471953095580748,0.32703994829039285,0.0>,0.05
    ,<-0.2513973747944036,0.34666613663797585,0.0>,0.05
    ,<-0.25158043516505374,0.3667551979054379,0.0>,0.05
    ,<-0.24797691056548998,0.3865350800425222,0.0>,0.05
    ,<-0.24096070802878009,0.4053893706042772,0.0>,0.05
    ,<-0.23097874147230374,0.4228654756551339,0.0>,0.05
    ,<-0.21849714928356462,0.4386603345599038,0.0>,0.05
    ,<-0.2039652902475339,0.4525956372476031,0.0>,0.05
    ,<-0.18779557009265546,0.4645918662518436,0.0>,0.05
    ,<-0.17035472996034837,0.47464698294783775,0.0>,0.05
    ,<-0.1519612406439351,0.4828212123878402,0.0>,0.05
    ,<-0.13288515741004303,0.4892271515676965,0.0>,0.05
    ,<-0.11334880472894446,0.4940245336709932,0.0>,0.05
    ,<-0.09352758408390097,0.49741889624746705,0.0>,0.05
    ,<-0.07355108889782268,0.4996635425970384,0.0>,0.05
    ,<-0.05350566283025007,0.5010644799544072,0.0>,0.05
    ,<-0.03344064437343463,0.5019881647569734,0.0>,0.05
    ,<-0.013381767301753065,0.5028676968937765,0.0>,0.05
    ,<0.006643801096331405,0.5042021096559562,0.0>,0.05
    ,<0.026568071726880665,0.5065460530389226,0.0>,0.05
    ,<0.04623063473811974,0.5104843169378889,0.0>,0.05
    ,<0.06532432773090323,0.5165832852719291,0.0>,0.05
    ,<0.08335699828415519,0.5253134236784723,0.0>,0.05
    ,<0.09965625429136481,0.5369474325493718,0.0>,0.05
    ,<0.11343855232769383,0.5514632301453621,0.0>,0.05
    ,<0.1239375650620903,0.5684963407792912,0.0>,0.05
    ,<0.1305590287375007,0.5873719667347154,0.0>,0.05
    ,<0.13301058633993137,0.60722132961142,0.0>,0.05
    ,<0.1313597783099311,0.6271524807096649,0.0>,0.05
    ,<0.1260021395473544,0.64642194206462,0.0>,0.05
    ,<0.1175587346573745,0.6645548685312265,0.0>,0.05
    ,<0.10674709159207614,0.6813858160844057,0.0>,0.05
    ,<0.09426956332560754,0.6970233881886921,0.0>,0.05
    ,<0.08074283660622092,0.71176299733944,0.0>,0.05
    ,<0.06666576096838044,0.7259765900541306,0.0>,0.05
    ,<0.052403434426143404,0.7400018141706024,0.0>,0.05
    ,<0.038160726197227705,0.7540440635464833,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
