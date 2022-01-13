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
    ,<-0.3063964101922081,0.262749125763663,0.0>,0.05
    ,<-0.30287335770546064,0.2824103840816362,0.0>,0.05
    ,<-0.29906110958471666,0.30196751295988844,0.0>,0.05
    ,<-0.29465638418254925,0.321354230872149,0.0>,0.05
    ,<-0.28929214709035256,0.3404596807924121,0.0>,0.05
    ,<-0.2825898234572761,0.35910989673178534,0.0>,0.05
    ,<-0.2742108928157264,0.3770536101023614,0.0>,0.05
    ,<-0.26390622370086386,0.3939629185957626,0.0>,0.05
    ,<-0.2515575399263585,0.4094556179672771,0.0>,0.05
    ,<-0.237202222996731,0.4231395296003824,0.0>,0.05
    ,<-0.22103288844886967,0.4346713257207847,0.0>,0.05
    ,<-0.20336795681695294,0.44381614058773844,0.0>,0.05
    ,<-0.1845977357288834,0.45049243796304234,0.0>,0.05
    ,<-0.16511873766808083,0.45479071007375044,0.0>,0.05
    ,<-0.14527322240241425,0.45696297950616327,0.0>,0.05
    ,<-0.12530897683309414,0.45738883435316163,0.0>,0.05
    ,<-0.10536725226835822,0.4565291946855293,0.0>,0.05
    ,<-0.08549723357364757,0.4548838252260543,0.0>,0.05
    ,<-0.0656876090548265,0.4529607643988754,0.0>,0.05
    ,<-0.045904944104333024,0.4512564533864762,0.0>,0.05
    ,<-0.026130922846831963,0.4502422705436578,0.0>,0.05
    ,<-0.006393074301041981,0.45035304043315816,0.0>,0.05
    ,<0.013214230799292803,0.4519748286690019,0.0>,0.05
    ,<0.032519808410188526,0.4554316244096443,0.0>,0.05
    ,<0.051276507238149596,0.4609711583163273,0.0>,0.05
    ,<0.06917164398186319,0.46875155582139,0.0>,0.05
    ,<0.0858445395948835,0.47883179498189804,0.0>,0.05
    ,<0.10090746193661065,0.4911679769442619,0.0>,0.05
    ,<0.11396575722789375,0.5056156377590809,0.0>,0.05
    ,<0.12463333710632706,0.5219361894358178,0.0>,0.05
    ,<0.13254123249525834,0.5398034994618985,0.0>,0.05
    ,<0.13734315550441464,0.5588041123369643,0.0>,0.05
    ,<0.13872673558780388,0.5784285541992361,0.0>,0.05
    ,<0.136437027117049,0.5980580562672051,0.0>,0.05
    ,<0.13031713365973377,0.6169550865819065,0.0>,0.05
    ,<0.12036670342711249,0.6342703699053169,0.0>,0.05
    ,<0.10681109930851232,0.6490822051344208,0.0>,0.05
    ,<0.09016160189752041,0.6604823279836811,0.0>,0.05
    ,<0.07123048390525046,0.6677044056834474,0.0>,0.05
    ,<0.05106999360535934,0.6702599674019102,0.0>,0.05
    ,<0.03083758297307541,0.6680353889039583,0.0>,0.05
    ,<0.01161976286307193,0.6613115221070138,0.0>,0.05
    ,<-0.005734075768071436,0.6506949229708551,0.0>,0.05
    ,<-0.020720224772362194,0.6369839300051612,0.0>,0.05
    ,<-0.033208379069958965,0.6210155446947978,0.0>,0.05
    ,<-0.043395520227113155,0.6035394708731067,0.0>,0.05
    ,<-0.05171261785645306,0.5851472981789984,0.0>,0.05
    ,<-0.05871327112401979,0.5662597067596109,0.0>,0.05
    ,<-0.06496314470535042,0.5471539198432421,0.0>,0.05
    ,<-0.07093618252561723,0.5280026452660511,0.0>,0.05
    ,<-0.07691594449541665,0.5088962464875257,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
