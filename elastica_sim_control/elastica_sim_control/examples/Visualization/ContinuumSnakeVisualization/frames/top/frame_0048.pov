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
    ,<-0.16673044536673104,0.5356652463872987,0.0>,0.05
    ,<-0.16985974070378645,0.515917063898059,0.0>,0.05
    ,<-0.17262852635996787,0.49612589260482653,0.0>,0.05
    ,<-0.17445443451816478,0.47623574459687973,0.0>,0.05
    ,<-0.17462576114872003,0.4562718275388671,0.0>,0.05
    ,<-0.1724055156284388,0.43643857278988335,0.0>,0.05
    ,<-0.16715308666834391,0.4171902047267827,0.0>,0.05
    ,<-0.15846060001473875,0.39923267864765966,0.0>,0.05
    ,<-0.14627196322341846,0.3834339534454807,0.0>,0.05
    ,<-0.13093496870316165,0.37065590000175563,0.0>,0.05
    ,<-0.11314939096859408,0.36156124683297747,0.0>,0.05
    ,<-0.09381593381903629,0.3564677296064502,0.0>,0.05
    ,<-0.07383767508517046,0.3553028470227134,0.0>,0.05
    ,<-0.05394654349684681,0.35766599551256195,0.0>,0.05
    ,<-0.034610321326237564,0.3629604199403628,0.0>,0.05
    ,<-0.016035887100244388,0.370541679156669,0.0>,0.05
    ,<0.00175228153670464,0.3798428301030688,0.0>,0.05
    ,<0.018805922281366714,0.3904471984157282,0.0>,0.05
    ,<0.035160827970674345,0.4021129793512513,0.0>,0.05
    ,<0.05077295420794464,0.4147669991913006,0.0>,0.05
    ,<0.0654803208270685,0.4284742465443406,0.0>,0.05
    ,<0.07898505150271246,0.4433815266049917,0.0>,0.05
    ,<0.09085783819966829,0.45963374535058754,0.0>,0.05
    ,<0.10057923056795064,0.47727286540194347,0.0>,0.05
    ,<0.10761682418829893,0.49615816147683856,0.0>,0.05
    ,<0.11152088586958994,0.5159418741870828,0.0>,0.05
    ,<0.11201489320378681,0.5361086479992556,0.0>,0.05
    ,<0.10904905822904183,0.5560652576394359,0.0>,0.05
    ,<0.10279490593289735,0.5752454844648056,0.0>,0.05
    ,<0.09358287207525405,0.5931873397317844,0.0>,0.05
    ,<0.08180774156677578,0.6095512901796787,0.0>,0.05
    ,<0.06784804675473524,0.6240819090677455,0.0>,0.05
    ,<0.0520314708767678,0.6365463707788734,0.0>,0.05
    ,<0.03464338894964095,0.6466771109344149,0.0>,0.05
    ,<0.015969579225355277,0.6541363839113224,0.0>,0.05
    ,<-0.003638066070150422,0.6585155961377561,0.0>,0.05
    ,<-0.023690013898288308,0.6593804148775742,0.0>,0.05
    ,<-0.04351072448983388,0.6563669742070266,0.0>,0.05
    ,<-0.06225054672736909,0.6493074181849251,0.0>,0.05
    ,<-0.07897436821518455,0.6383330544100023,0.0>,0.05
    ,<-0.09280863429852655,0.6239129429951729,0.0>,0.05
    ,<-0.10310145835790761,0.6068030473173849,0.0>,0.05
    ,<-0.10954050079079837,0.5879141252736244,0.0>,0.05
    ,<-0.11218926567159707,0.5681403241514952,0.0>,0.05
    ,<-0.11143721800205025,0.548205277117786,0.0>,0.05
    ,<-0.10789258232870035,0.528570802405042,0.0>,0.05
    ,<-0.10226065228086428,0.5094245056487546,0.0>,0.05
    ,<-0.09524075078875284,0.49073419049526606,0.0>,0.05
    ,<-0.08745166611699863,0.4723411567835295,0.0>,0.05
    ,<-0.07937355949894753,0.45406221175827094,0.0>,0.05
    ,<-0.07128499579258166,0.43577651793806926,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
