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
    ,<0.02152562282293174,-0.019476705009485628,0.0>,0.05
    ,<0.01477415338752376,-0.0006510018905300077,0.0>,0.05
    ,<0.008016863506652198,0.018172056654949236,0.0>,0.05
    ,<0.0012623431111622532,0.03699554551631843,0.0>,0.05
    ,<-0.005476320378467353,0.055824131569228615,0.0>,0.05
    ,<-0.012182017768284386,0.07466385947733412,0.0>,0.05
    ,<-0.01883408498473624,0.09352190955719233,0.0>,0.05
    ,<-0.025408760401374318,0.11240632665084696,0.0>,0.05
    ,<-0.031879633019790556,0.1313257221206599,0.0>,0.05
    ,<-0.03821808294011257,0.15028895376510543,0.0>,0.05
    ,<-0.044393716334263505,0.16930479066293852,0.0>,0.05
    ,<-0.05037479759665809,0.18838157169832023,0.0>,0.05
    ,<-0.05612868142702168,0.2075268678087698,0.0>,0.05
    ,<-0.061622247258897304,0.22674715883196012,0.0>,0.05
    ,<-0.06682233767768528,0.2460475361989804,0.0>,0.05
    ,<-0.07169620130249243,0.26543144261621626,0.0>,0.05
    ,<-0.07621193910310799,0.2849004592792544,0.0>,0.05
    ,<-0.08033895138871211,0.30445415005766324,0.0>,0.05
    ,<-0.08404838087036948,0.32408997048047894,0.0>,0.05
    ,<-0.08731354541795018,0.3438032472615054,0.0>,0.05
    ,<-0.09011035256621494,0.3635872315841416,0.0>,0.05
    ,<-0.09241768663173305,0.38343322650546924,0.0>,0.05
    ,<-0.09421775861872649,0.4033307857628109,0.0>,0.05
    ,<-0.09549640901879665,0.42326797812967876,0.0>,0.05
    ,<-0.09624335419892249,0.44323170845103504,0.0>,0.05
    ,<-0.09645236831897255,0.4632080837798588,0.0>,0.05
    ,<-0.09612139455856494,0.4831828108181479,0.0>,0.05
    ,<-0.09525258174121148,0.5031416092943785,0.0>,0.05
    ,<-0.09385224505257027,0.5230706250966378,0.0>,0.05
    ,<-0.09193075226056174,0.5429568269906004,0.0>,0.05
    ,<-0.08950233944855734,0.5627883715805841,0.0>,0.05
    ,<-0.08658486257051219,0.5825549227594226,0.0>,0.05
    ,<-0.08319949296086637,0.6022479141235929,0.0>,0.05
    ,<-0.0793703661610646,0.6218607455465304,0.0>,0.05
    ,<-0.07512419399428974,0.6413889081236428,0.0>,0.05
    ,<-0.07048984972650231,0.6608300348382841,0.0>,0.05
    ,<-0.06549793544854134,0.6801838773695167,0.0>,0.05
    ,<-0.060180339602781956,0.6994522123142716,0.0>,0.05
    ,<-0.054569790996154804,0.7186386826050866,0.0>,0.05
    ,<-0.04869941384703114,0.7377485819831615,0.0>,0.05
    ,<-0.04260228656927644,0.7567885919828587,0.0>,0.05
    ,<-0.036311005255650425,0.7757664819765204,0.0>,0.05
    ,<-0.029857251316609972,0.7946907834168098,0.0>,0.05
    ,<-0.02327136156082145,0.8135704495112932,0.0>,0.05
    ,<-0.016581898236774393,0.8324145111867202,0.0>,0.05
    ,<-0.009815216219096921,0.8512317393641268,0.0>,0.05
    ,<-0.0029950246113629704,0.8700303222766266,0.0>,0.05
    ,<0.003858059490770148,0.8888175648198572,0.0>,0.05
    ,<0.010726966540951925,0.907599614721397,0.0>,0.05
    ,<0.01759863982297702,0.9263812176386077,0.0>,0.05
    ,<0.024464509720634756,0.9451655001323579,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }