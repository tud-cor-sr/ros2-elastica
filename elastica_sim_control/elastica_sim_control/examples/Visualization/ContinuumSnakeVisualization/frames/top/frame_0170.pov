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
    ,<0.07775331253964009,0.022620858781793457,0.0>,0.05
    ,<0.06577500419696101,0.03863624263699785,0.0>,0.05
    ,<0.05378836198190679,0.054643680238337766,0.0>,0.05
    ,<0.041806301847364884,0.07065281044814808,0.0>,0.05
    ,<0.029848665913644996,0.08667837414007751,0.0>,0.05
    ,<0.017941617012448223,0.10273959699269475,0.0>,0.05
    ,<0.006117084534288517,0.11885948999747506,0.0>,0.05
    ,<-0.005587737422685239,0.13506406011369285,0.0>,0.05
    ,<-0.017130846308249046,0.1513814311263407,0.0>,0.05
    ,<-0.028465855607078795,0.1678408814819813,0.0>,0.05
    ,<-0.039542407502352245,0.18447181195985526,0.0>,0.05
    ,<-0.05030659833288504,0.20130266177304193,0.0>,0.05
    ,<-0.060701444215789914,0.2183597972657162,0.0>,0.05
    ,<-0.07066741463611624,0.23566640284412887,0.0>,0.05
    ,<-0.08014305957017462,0.25324140900330233,0.0>,0.05
    ,<-0.08906575062892716,0.2710984969528161,0.0>,0.05
    ,<-0.09737254879321522,0.28924522285221865,0.0>,0.05
    ,<-0.10500120084531177,0.3076823063549801,0.0>,0.05
    ,<-0.11189125416641914,0.32640312728367366,0.0>,0.05
    ,<-0.11798526607562192,0.3453934701499752,0.0>,0.05
    ,<-0.12323007053135532,0.3646315484463576,0.0>,0.05
    ,<-0.12757805319500257,0.38408832908396473,0.0>,0.05
    ,<-0.13098837701556806,0.4037281624293728,0.0>,0.05
    ,<-0.1334280959304258,0.4235097059967156,0.0>,0.05
    ,<-0.13487309493828034,0.4433871113713776,0.0>,0.05
    ,<-0.135308801085761,0.46331142608001696,0.0>,0.05
    ,<-0.1347306215603795,0.48323214673266657,0.0>,0.05
    ,<-0.13314408114809706,0.5030988485292753,0.0>,0.05
    ,<-0.13056465024861244,0.5228628104540879,0.0>,0.05
    ,<-0.12701727449783423,0.5424785558440665,0.0>,0.05
    ,<-0.12253563574692584,0.5619052344833658,0.0>,0.05
    ,<-0.11716118977982273,0.5811077841629203,0.0>,0.05
    ,<-0.11094203723029201,0.6000578254161335,0.0>,0.05
    ,<-0.1039316898184431,0.6187342611507971,0.0>,0.05
    ,<-0.09618779408160612,0.6371235713232823,0.0>,0.05
    ,<-0.08777086970273548,0.6552198099426918,0.0>,0.05
    ,<-0.07874311033089454,0.6730243262354734,0.0>,0.05
    ,<-0.06916728276998746,0.6905452428904814,0.0>,0.05
    ,<-0.05910574702292041,0.7077967316192492,0.0>,0.05
    ,<-0.048619606294060785,0.7247981299336955,0.0>,0.05
    ,<-0.0377679838321561,0.7415729435479096,0.0>,0.05
    ,<-0.026607413303862185,0.7581477768389386,0.0>,0.05
    ,<-0.01519132176981614,0.7745512300976916,0.0>,0.05
    ,<-0.003569579529254364,0.7908127975537182,0.0>,0.05
    ,<0.008211910901693261,0.8069617948901221,0.0>,0.05
    ,<0.020111613821117286,0.8230263395197376,0.0>,0.05
    ,<0.03209277167050257,0.839032401382863,0.0>,0.05
    ,<0.04412384826125412,0.8550029364049678,0.0>,0.05
    ,<0.05617901741336594,0.8709571088076578,0.0>,0.05
    ,<0.06823870349655707,0.8869096019420623,0.0>,0.05
    ,<0.0802901727779596,0.9028700099094498,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
