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
    ,<-0.2080335741101552,0.41281157583410444,0.0>,0.05
    ,<-0.19993506357472926,0.3945335451212167,0.0>,0.05
    ,<-0.1915035858049176,0.3764242024092286,0.0>,0.05
    ,<-0.1822328351459351,0.3587457526580597,0.0>,0.05
    ,<-0.17155745231505756,0.34189105733138786,0.0>,0.05
    ,<-0.15898815340543496,0.3264045535525092,0.0>,0.05
    ,<-0.14424278557447456,0.312971597179922,0.0>,0.05
    ,<-0.12734954483710884,0.30234727345432255,0.0>,0.05
    ,<-0.1086870681656062,0.2952255166710767,0.0>,0.05
    ,<-0.08893285666288056,0.2920853722373579,0.0>,0.05
    ,<-0.06892192776777067,0.2930732535081466,0.0>,0.05
    ,<-0.04945772302963936,0.2979711169686762,0.0>,0.05
    ,<-0.03114119279433487,0.30626223790832097,0.0>,0.05
    ,<-0.01427463826852091,0.31726195657286427,0.0>,0.05
    ,<0.001138612635523036,0.3302586238249454,0.0>,0.05
    ,<0.015314664275303366,0.34462102478345935,0.0>,0.05
    ,<0.02856562088110014,0.3598585634640324,0.0>,0.05
    ,<0.04118278724718164,0.37563627388644905,0.0>,0.05
    ,<0.0533567006791631,0.3917663620311591,0.0>,0.05
    ,<0.06513592749833737,0.4081938207091588,0.0>,0.05
    ,<0.07641635625630026,0.4249758776701898,0.0>,0.05
    ,<0.08695161320595132,0.44224635270724827,0.0>,0.05
    ,<0.09638128666983456,0.46015831380507016,0.0>,0.05
    ,<0.10428313685323772,0.4788105057420734,0.0>,0.05
    ,<0.11024057143249243,0.4981871829963039,0.0>,0.05
    ,<0.11391104656207836,0.5181371780881439,0.0>,0.05
    ,<0.11508479043357024,0.53839721880315,0.0>,0.05
    ,<0.11371648026868432,0.5586492767469543,0.0>,0.05
    ,<0.10991687470381349,0.5785879594473786,0.0>,0.05
    ,<0.10390438153413732,0.5979694491529164,0.0>,0.05
    ,<0.09592921677006831,0.6166208451852123,0.0>,0.05
    ,<0.08620278847561075,0.6344106074634132,0.0>,0.05
    ,<0.07485959711079568,0.6512015852321544,0.0>,0.05
    ,<0.0619512138568716,0.6668032724558338,0.0>,0.05
    ,<0.04747011337250702,0.6809327914931939,0.0>,0.05
    ,<0.03140199214441152,0.6931922987292644,0.0>,0.05
    ,<0.013802203187006394,0.7030741536145872,0.0>,0.05
    ,<-0.005118689381995681,0.7100098301680934,0.0>,0.05
    ,<-0.024934695304489114,0.7134650867989557,0.0>,0.05
    ,<-0.04500568616145313,0.7130552614209097,0.0>,0.05
    ,<-0.06455043579385963,0.7086477598505984,0.0>,0.05
    ,<-0.08277646311826756,0.7004142389432572,0.0>,0.05
    ,<-0.09902783162878669,0.6888086611485479,0.0>,0.05
    ,<-0.1129033880393315,0.6744761858119656,0.0>,0.05
    ,<-0.12431097883673062,0.6581258538696181,0.0>,0.05
    ,<-0.13345006014846025,0.6404114620524528,0.0>,0.05
    ,<-0.1407397072243701,0.6218553243858967,0.0>,0.05
    ,<-0.1467190606856021,0.6028266445491992,0.0>,0.05
    ,<-0.15194174430898302,0.5835626593478905,0.0>,0.05
    ,<-0.15687325363648577,0.5642058687382693,0.0>,0.05
    ,<-0.16179068734493865,0.544828413160463,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
