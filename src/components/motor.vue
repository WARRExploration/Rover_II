<template>
    <div class="motorcontroller">
        <div class="motordisplay">
            <div class="progress" :style="{height: (Math.abs(speed / max_speed) * 50) + '%'}" :class="{negative: speed < 0}"></div>
            <div class="speedlabel">{{ speed }}</div>
        </div>
        <div class="motorslider">
            <vue-slider ref="slider" v-model="value" v-bind="slider_conf" :disabled="!active" @drag-end="resetValue()">
                <span slot="label" slot-scope="{ label, active }" :class="['custom-label', { active }]" v-if="label % step === 0">
                    {{ label }}
                </span>
            </vue-slider>
        </div>
        <div class="label"  @click.ctrl="value = 0">{{ value }}</div>
    </div>
</template>

<script>
import vueSlider from "vue-slider-component";

export default {
  components: {
    vueSlider
  },
  props: ["config", "address", "motor", "active", "value", "max_speed", "snapback"],
  data() {
    return {
      //   speed: 0,
      current: 0,
      max_current: 0,
      framerate: 15,
      topic: null,
      step: 1000,
      slider_conf: {
        tooltip: false,
        direction: "vertical",
        height: "100%",
        width: "6",
        piecewiseLabel: true,
        min: -3200,
        max: 3200,
        interval: 100,
        clickable: false,
        processStyle: {
          display: "none"
        }
      }
    };
  },
  created() {
    this.topic = new ROSLIB.Topic({
      ros: this.config.ros,
      name: "/motor_driver",
      messageType: "exploration_rover_i/tcmc"
    });

    this.slider_conf.min = -this.max_speed;
    this.slider_conf.max = this.max_speed;
    this.slider_conf.interval = 2 * this.max_speed / 5;
  },
  computed: {
    speed() {
      return this.value;
    }
  },
  watch: {
    active: {
      immediate: true,
      handler(newVal, oldVal) {
        if (this.active) {
          if (!this.interval)
            this.interval = setInterval(this.update, 1000.0 / this.framerate);
        } else {
          if (this.interval) {
            clearTimeout(this.interval);
            delete this.interval;
          }
        }
      }
    }
  },
  methods: {
    resetValue() {
      if (this.snapback) {
        this.value = 0;
      }
    },
    update() {
      var msg = new ROSLIB.Message({
        address: this.address,
        command: 1,
        type: 0,
        motor: this.motor,
        value: parseInt(this.value)
      });
      this.topic.publish(msg);
      // console.log(msg);
    }
  }
};
</script>

<style lang="scss">
.motorcontroller {
  height: 50vh;
  display: flex;
  align-items: center;
  flex-direction: column;
}

.motordisplay {
  flex-grow: 1;
  position: relative;
  background: #333333;
  min-width: 8em;
  border-radius: 0.5em;
  margin: 0.5em;
  overflow: hidden;
  display: flex;
  align-items: center;
  justify-content: space-around;
}

.speedlabel {
  position: relative;
  z-index: 2;
  background-color: #222222;
  color: #fff;
  padding: 0.5em;
  margin: 0.5em;
  border-radius: 0.75em;
  min-width: 4em;
  text-align: center;
}

.progress {
  z-index: 1;
  position: absolute;
  width: 100%;
  background-color: #008800;
  bottom: 50%;
}

.progress.negative {
  bottom: auto;
  top: 50%;
  background-color: #880000;
}

.motorslider {
  position: relative;
  flex-grow: 1;
  margin: 0.5em;

  & > div {
    position: absolute;
    transform: translateX(-50%);
  }
}

.custom-label {
  margin-left: 1em;
}

.vue-slider-component .vue-slider-piecewise {
  top: -0.35em;
}

.label {
  background-color: #333333;
  color: #fff;
  padding: 0.5em;
  margin: 0.5em;
  border-radius: 0.75em;
  min-width: 5em;
  text-align: center;
}
</style>
