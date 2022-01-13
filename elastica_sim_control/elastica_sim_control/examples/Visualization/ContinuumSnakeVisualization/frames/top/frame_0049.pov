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
    ,<-0.16412784342195302,0.5403224330124916,0.0>,0.05
    ,<-0.16777470687122797,0.5206630837167543,0.0>,0.05
    ,<-0.17106264980376745,0.5009510404922815,0.0>,0.05
    ,<-0.1734104758289777,0.4811149907695351,0.0>,0.05
    ,<-0.174105267232922,0.4611615069420589,0.0>,0.05
    ,<-0.17240387053358924,0.44127601692188234,0.0>,0.05
    ,<-0.16765284503319106,0.42189684850055187,0.0>,0.05
    ,<-0.15942509705391059,0.4037209836473326,0.0>,0.05
    ,<-0.14764176904657117,0.38761748568455395,0.0>,0.05
    ,<-0.1326292332821144,0.3744604855760268,0.0>,0.05
    ,<-0.11507247969215513,0.36493420886031014,0.0>,0.05
    ,<-0.09586769917102805,0.3593825684637835,0.0>,0.05
    ,<-0.07592434952261921,0.3577583645465021,0.0>,0.05
    ,<-0.05598928409320939,0.3596809282940996,0.0>,0.05
    ,<-0.03654970295294839,0.3645660434909456,0.0>,0.05
    ,<-0.01783225498507383,0.3717750007920186,0.0>,0.05
    ,<0.00012154420351887039,0.38074210252460955,0.0>,0.05
    ,<0.01735050767603226,0.3910500629562125,0.0>,0.05
    ,<0.033881562766891006,0.4024564841191174,0.0>,0.05
    ,<0.049664955854812017,0.41488845582069933,0.0>,0.05
    ,<0.06453479393004014,0.4284120415709392,0.0>,0.05
    ,<0.078189565794731,0.44317524505443023,0.0>,0.05
    ,<0.09019529386492998,0.4593231398699405,0.0>,0.05
    ,<0.10002620165472749,0.4768953948834695,0.0>,0.05
    ,<0.10714245210778912,0.49574532738543897,0.0>,0.05
    ,<0.11108737205549797,0.5155151570698292,0.0>,0.05
    ,<0.11157992552588882,0.5356761691254983,0.0>,0.05
    ,<0.10856964344556726,0.5556201915901798,0.0>,0.05
    ,<0.10223150101009086,0.5747666424786826,0.0>,0.05
    ,<0.09290275115955848,0.5926415149610378,0.0>,0.05
    ,<0.08098718712463575,0.6088963632156417,0.0>,0.05
    ,<0.06687372543821511,0.6232697706062942,0.0>,0.05
    ,<0.050901662370460204,0.6355253263951746,0.0>,0.05
    ,<0.033369603852174426,0.6453940014156,0.0>,0.05
    ,<0.014578700014671176,0.6525390329346125,0.0>,0.05
    ,<-0.005101534542436599,0.656556330293813,0.0>,0.05
    ,<-0.025162600685532565,0.6570211863265718,0.0>,0.05
    ,<-0.04491104379888439,0.6535859051960782,0.0>,0.05
    ,<-0.06348397951498112,0.646105446235504,0.0>,0.05
    ,<-0.07994133714945607,0.6347385271090864,0.0>,0.05
    ,<-0.09341500635183721,0.6199821164582004,0.0>,0.05
    ,<-0.1032684984498677,0.6026155420850776,0.0>,0.05
    ,<-0.10921164276949634,0.5835640854310123,0.0>,0.05
    ,<-0.11133197218031589,0.5637254904552861,0.0>,0.05
    ,<-0.11003991243665807,0.5438166870387356,0.0>,0.05
    ,<-0.10595821327563854,0.5242852088937022,0.0>,0.05
    ,<-0.09979921845756223,0.5053003530320148,0.0>,0.05
    ,<-0.09226293459018686,0.4868108589260736,0.0>,0.05
    ,<-0.08396492849485385,0.4686407090225646,0.0>,0.05
    ,<-0.07538109553324064,0.45059308747248206,0.0>,0.05
    ,<-0.06678717117569975,0.43253917414375165,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
