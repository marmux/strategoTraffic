/*

*/
strategy Opt =minE (totalJammedCars) [<=2*Horizon]: <> trafficLights.DONE

/*

*/
simulate 1 [<=2*Horizon] { signal[1], signal[2], signal[3], signal[4], signal[5], signal[6]} under Opt

