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
    ,<-0.12342519729839548,0.07799366497909209,0.0>,0.05
    ,<-0.10430779412776722,0.0838340815702385,0.0>,0.05
    ,<-0.08533287451119055,0.09006195102145284,0.0>,0.05
    ,<-0.06667889174988798,0.09715823036981512,0.0>,0.05
    ,<-0.04862334518982851,0.10565765738409585,0.0>,0.05
    ,<-0.031563174022931566,0.11603538589912388,0.0>,0.05
    ,<-0.01600798695669732,0.12860333550299483,0.0>,0.05
    ,<-0.0025238726845513287,0.1434333127384705,0.0>,0.05
    ,<0.008367761539719685,0.16032944227116316,0.0>,0.05
    ,<0.016309046893858654,0.17886838851130055,0.0>,0.05
    ,<0.021199378652888942,0.19850224956002208,0.0>,0.05
    ,<0.023229909687006008,0.21869261440292645,0.0>,0.05
    ,<0.02285103880366353,0.2390279896579292,0.0>,0.05
    ,<0.020693817029407577,0.25928517652322725,0.0>,0.05
    ,<0.017477393196535188,0.2794208191818904,0.0>,0.05
    ,<0.013923319703491761,0.2995068992084332,0.0>,0.05
    ,<0.010678021170078777,0.31964265591760527,0.0>,0.05
    ,<0.008252468752730016,0.3398839258980933,0.0>,0.05
    ,<0.0069801985151788735,0.3602165648893715,0.0>,0.05
    ,<0.006993773642134719,0.3805742313678912,0.0>,0.05
    ,<0.008226967757170018,0.40088179149592595,0.0>,0.05
    ,<0.0104466402050365,0.4210970838772389,0.0>,0.05
    ,<0.013311084503953206,0.44122873888015024,0.0>,0.05
    ,<0.01644792348541052,0.46132288374049424,0.0>,0.05
    ,<0.019525501396494997,0.48143315273924125,0.0>,0.05
    ,<0.02230112623398296,0.5015958389586811,0.0>,0.05
    ,<0.024649142218246337,0.5218208704053653,0.0>,0.05
    ,<0.026568748284365947,0.5420975933252755,0.0>,0.05
    ,<0.028169646994922398,0.5624069402263909,0.0>,0.05
    ,<0.02963423460545689,0.5827307985300577,0.0>,0.05
    ,<0.031156753760243672,0.6030549435949643,0.0>,0.05
    ,<0.03287840997147759,0.6233686932576283,0.0>,0.05
    ,<0.034840996401383544,0.643666909880831,0.0>,0.05
    ,<0.03695983912077549,0.663955526123746,0.0>,0.05
    ,<0.039015360505030426,0.684254746311304,0.0>,0.05
    ,<0.040662591660814325,0.7045907214113853,0.0>,0.05
    ,<0.04145943272714085,0.7249698888351602,0.0>,0.05
    ,<0.0409174870801703,0.7453392713147113,0.0>,0.05
    ,<0.038570210771047574,0.7655495649260844,0.0>,0.05
    ,<0.034040839160054355,0.7853406830159726,0.0>,0.05
    ,<0.027101714890588515,0.8043628828553088,0.0>,0.05
    ,<0.01771161967289317,0.8222325358085512,0.0>,0.05
    ,<0.0060178931359199635,0.8386097131682562,0.0>,0.05
    ,<-0.00767923518155125,0.8532724129818694,0.0>,0.05
    ,<-0.022990178310740236,0.8661666268011506,0.0>,0.05
    ,<-0.039505160506697654,0.8774170076478646,0.0>,0.05
    ,<-0.056850793009606757,0.8873010652703368,0.0>,0.05
    ,<-0.0747194946809839,0.8961931527542891,0.0>,0.05
    ,<-0.09287791430809447,0.9044923007482759,0.0>,0.05
    ,<-0.11116296632747627,0.9125390795105132,0.0>,0.05
    ,<-0.12948939661088327,0.9205288241133187,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
