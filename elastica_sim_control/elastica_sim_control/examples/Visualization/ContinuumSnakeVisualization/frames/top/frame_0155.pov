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
    ,<0.11741777333676968,0.07748869893789005,0.0>,0.05
    ,<0.1016844163908786,0.08983473670883352,0.0>,0.05
    ,<0.08594265116472712,0.10216710448095558,0.0>,0.05
    ,<0.07020623756689715,0.11450328385021256,0.0>,0.05
    ,<0.05449645982110049,0.12687014932645935,0.0>,0.05
    ,<0.03884163450374123,0.13930292778366157,0.0>,0.05
    ,<0.023276757135509467,0.15184402510862677,0.0>,0.05
    ,<0.007843292677575322,0.1645417010810961,0.0>,0.05
    ,<-0.007410904052674316,0.17744858165882632,0.0>,0.05
    ,<-0.022431571502311513,0.19062000563942377,0.0>,0.05
    ,<-0.03715797863912908,0.20411220993476017,0.0>,0.05
    ,<-0.051522882413993325,0.21798036705418777,0.0>,0.05
    ,<-0.06545258252328111,0.23227649958980243,0.0>,0.05
    ,<-0.07886715718884876,0.24704731145069733,0.0>,0.05
    ,<-0.09168096302791261,0.2623319929922293,0.0>,0.05
    ,<-0.10380347391234143,0.2781600772285737,0.0>,0.05
    ,<-0.11514051585712459,0.29454944394178306,0.0>,0.05
    ,<-0.12559592793854538,0.3115045857762798,0.0>,0.05
    ,<-0.13507364309064154,0.32901526091393996,0.0>,0.05
    ,<-0.14348014013127325,0.34705565764888696,0.0>,0.05
    ,<-0.1507271730312549,0.36558418347435023,0.0>,0.05
    ,<-0.15673464067020818,0.3845439637226192,0.0>,0.05
    ,<-0.1614334261357728,0.40386409263689926,0.0>,0.05
    ,<-0.16476801479605105,0.4234616257422622,0.0>,0.05
    ,<-0.16669869968728201,0.4432442422843873,0.0>,0.05
    ,<-0.16720320316595405,0.46311344690986656,0.0>,0.05
    ,<-0.16627758460992467,0.4829681294578756,0.0>,0.05
    ,<-0.16393636079273938,0.5027082669227165,0.0>,0.05
    ,<-0.16021183162899302,0.5222385386145652,0.0>,0.05
    ,<-0.15515267069051317,0.5414716350179767,0.0>,0.05
    ,<-0.14882189861088502,0.5603310725247281,0.0>,0.05
    ,<-0.14129440118994954,0.5787533740924425,0.0>,0.05
    ,<-0.13265417798293141,0.5966895340933496,0.0>,0.05
    ,<-0.12299151035476305,0.6141057453394899,0.0>,0.05
    ,<-0.11240022197038155,0.6309834210228321,0.0>,0.05
    ,<-0.10097517396628847,0.647318587968787,0.0>,0.05
    ,<-0.08881009707068277,0.6631207573290321,0.0>,0.05
    ,<-0.07599581967246659,0.6784113938518838,0.0>,0.05
    ,<-0.06261890952734879,0.6932221064400297,0.0>,0.05
    ,<-0.04876071109655254,0.7075926740539139,0.0>,0.05
    ,<-0.034496733140050466,0.7215690050987126,0.0>,0.05
    ,<-0.019896322432339762,0.7352011094690161,0.0>,0.05
    ,<-0.005022549774525396,0.7485411424727327,0.0>,0.05
    ,<0.010067768126119892,0.7616415620506752,0.0>,0.05
    ,<0.02532398320735924,0.7745534251399177,0.0>,0.05
    ,<0.040701531201660486,0.7873248372858643,0.0>,0.05
    ,<0.05616189945064189,0.799999560126801,0.0>,0.05
    ,<0.07167268602060085,0.8126157747488796,0.0>,0.05
    ,<0.0872077733480443,0.8252049927781983,0.0>,0.05
    ,<0.10274762460019048,0.8377911018210107,0.0>,0.05
    ,<0.11827969500535128,0.8503895251271067,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
