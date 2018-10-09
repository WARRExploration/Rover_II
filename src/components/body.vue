<template>

    <tabs :class="{hide: config.fullscreen}">
      <tab name="Cameras">
        <div style="display: flex; width: 100%;">
          <CameraStream style="flex-grow: 1;" :src="'http://' + config.url + ':8080/stream?topic=/left/image_raw&quality=30&width=500&height=400'"></CameraStream>
          <CameraStream style="flex-grow: 1;" :src="'http://' + config.url + ':8080/stream?topic=/right/image_raw&quality=30&width=500&height=400'"></CameraStream>
          <!-- <CameraStream style="flex-grow: 1;" :src="'http://' + config.url + ':8080/stream?topic=/disparity_image&quality=30&width=500&height=400'"></CameraStream> -->
        </div>
      </tab>

      <tab name="Motors">
        <div style="display: flex; width: 100%;">
          <MotorSlider v-for="motor in motors" :key="motor.address" style="flex-grow: 1;" :active="active" v-bind="motor" :config="config"></MotorSlider>
          
          <div class="motorcontroller" style="flex-grow: 1;" >
            <div class="motordisplay">
                <button @click="active = !active" style="margin: 0.5em; padding: 0.5em">activate</button>
            </div>
            <div class="motorslider">
                <vue-slider ref="slider" v-model="master_value" v-bind="slider_conf" :disabled="!active">
                    <span slot="label" slot-scope="{ label, active }" :class="['custom-label', { active }]" v-if="label % 1000 === 0">
                        {{ label }}
                    </span>
                </vue-slider>
            </div>
            <div class="label"  @click.ctrl="master_value = 0">{{ master_value }}</div>
          </div>
        </div>
      </tab>

      <tab name="Arm">
        <div style="display: flex; width: 100%;">
          <div class="motorcontroller" style="flex-grow: 1;" >
            <div class="motordisplay">
                <div class="progress" :style="{height: (Math.abs(linear_pos / 1024) * 100) + '%', bottom: 0}"></div>
                <div class="speedlabel">{{ linear_pos }}</div>
            </div>
            <div class="motorslider">
                <vue-slider ref="slider" v-model="linear_value" v-bind="linear_conf" :disabled="false">
                    <span slot="label" slot-scope="{ label, active }" :class="['custom-label', { active }]" v-if="label % 128 === 0">
                        {{ label }}
                    </span>
                </vue-slider>
            </div>
            <div class="label"  @click.ctrl="linear_value = 0">{{ linear_value }}</div>
          </div>

          <MotorSlider v-for="arm in arm_joint" :key="arm.address" style="flex-grow: 1;" :active="true" v-bind="arm" :config="config"></MotorSlider>
        </div>
      </tab>
    </tabs>

</template>

<script>
import Vue from "vue";
import { Tabs, Tab } from "vue-tabs-component";
import CameraStream from "./camera_stream.vue";
import Motor from "./motor.vue";
import vueSlider from "vue-slider-component";

Vue.component("tabs", Tabs);
Vue.component("tab", Tab);

Vue.component("CameraStream", CameraStream);
Vue.component("MotorSlider", Motor);
Vue.component("vue-slider", vueSlider);

export default {
  props: ["config"],
  data() {
    return {
      motors: [
        { address: 1, motor: 0, value: 0, max_speed: 3200 },
        { address: 2, motor: 0, value: 0, max_speed: 3200 },
        { address: 3, motor: 0, value: 0, max_speed: 3200 },
        { address: 4, motor: 0, value: 0, max_speed: 3200 }
      ],

      arm_joint: [
        { address: 5, motor: 0, value: 0, snapback: true, max_speed: 30000 },
        { address: 5, motor: 1, value: 0, snapback: true, max_speed: 30000 },
        { address: 5, motor: 2, value: 0, snapback: true, max_speed: 30000 },
        { address: 5, motor: 3, value: 0, snapback: true, max_speed: 300000 }
      ],
      active: false,
      master_value: 0,
      linear_value: 512,
      framerate: 15,
      slider_conf: {
        tooltip: false,
        direction: "vertical",
        height: "100%",
        width: "6",
        piecewiseLabel: true,
        min: -3200,
        max: 3200,
        interval: 100,
        processStyle: {
          display: "none"
        }
      },
      linear_conf: {
        tooltip: false,
        direction: "vertical",
        height: "100%",
        width: "6",
        piecewiseLabel: true,
        min: 0,
        max: 1023,
        interval: 1,
        processStyle: {
          display: "none"
        }
      }
    };
  },
  computed: {
    linear_pos() {
      return this.  linear_value;
    }
  },
  watch: {
    master_value(newVal, oldVal) {
      this.motors.forEach(element => {
        element.value = newVal;
      });
    }
  },
  created() {
    var topic = new ROSLIB.Topic({
      ros: this.config.ros,
      name: "/motor_driver",
      messageType: "exploration_rover_i/tcmc"
    });

    // acceleration
    for (let i = 0; i < 4; i++) {
      var msg = new ROSLIB.Message({
        address: 5,
        command: 5,
        type: 5,
        motor: i,
        value: 800000
      });
      topic.publish(msg);
    }

    // run current
    for (let i = 0; i < 4; i++) {
      var msg = new ROSLIB.Message({
        address: 5,
        command: 5,
        type: 6,
        motor: i,
        value: 255
      });
      topic.publish(msg);
    }

    // standby current
    for (let i = 0; i < 2; i++) {
      var msg = new ROSLIB.Message({
        address: 5,
        command: 5,
        type: 7,
        motor: i,
        value: 255
      });
      topic.publish(msg);
    }
    for (let i = 2; i < 4; i++) {
      var msg = new ROSLIB.Message({
        address: 5,
        command: 5,
        type: 7,
        motor: i,
        value: 180
      });
      topic.publish(msg);
    }

    setInterval(this.update, 1000.0 / this.framerate);
    this.topic = new ROSLIB.Topic({
      ros: this.config.ros,
      name: "/linear_motor_driver",
      messageType: "std_msg/Int16"
    });
  },
  methods: {
    update() {
      var msg = new ROSLIB.Message({
        data: parseInt(this.linear_value)
      });
      this.topic.publish(msg);
      // console.log(msg);
    }
  }
};
</script>

<style lang="css">
.tabs-component {
  display: flex;
  align-items: stretch;
}

.tabs-component-tabs {
  display: flex;
  flex-direction: column;

  padding: 0;
  margin: 0;
  border-right: #0a243f solid 0.3em;
  background: #333;
}

.hide .tabs-component-tabs {
  display: none;
}

.tabs-component-tab {
  color: #999;
  font-size: 1em;
  font-weight: 600;
  list-style: none;
  padding-right: 2em;
}

.tabs-component-tab.is-active {
  color: #fff;
  background: #0a243f;
}

.tabs-component-tab-a {
  align-items: center;
  color: inherit;
  display: flex;
  padding: 0.75em 1em;
  text-decoration: none;
}

.tabs-component-panels {
  flex-grow: 1;
}
</style>
