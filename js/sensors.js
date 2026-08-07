
export class SensorHub {
  constructor(){
    this.heading = null;
    this.gravity = [0,0,-9.81];
    this.rotationRate = [0,0,0];
    this.orientation = {alpha:0,beta:0,gamma:0};
    this.live = false;
  }
  async start(){
    if (typeof DeviceMotionEvent !== "undefined" &&
        typeof DeviceMotionEvent.requestPermission === "function") {
      const result = await DeviceMotionEvent.requestPermission();
      if (result !== "granted") throw new Error("Motion permission denied");
    }
    if (typeof DeviceOrientationEvent !== "undefined" &&
        typeof DeviceOrientationEvent.requestPermission === "function") {
      const result = await DeviceOrientationEvent.requestPermission();
      if (result !== "granted") throw new Error("Orientation permission denied");
    }
    addEventListener("devicemotion", e => {
      const g=e.accelerationIncludingGravity||{};
      const r=e.rotationRate||{};
      this.gravity=[g.x||0,g.y||0,g.z||0];
      this.rotationRate=[r.alpha||0,r.beta||0,r.gamma||0];
      this.live=true;
    },{passive:true});
    addEventListener("deviceorientation", e => {
      if(Number.isFinite(e.webkitCompassHeading)) this.heading=e.webkitCompassHeading;
      else if(Number.isFinite(e.alpha)) this.heading=(360-e.alpha)%360;
      this.orientation={alpha:e.alpha||0,beta:e.beta||0,gamma:e.gamma||0};
    },{passive:true});
  }
}
