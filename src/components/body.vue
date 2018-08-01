<template>

    <tabs>
      <tab name="Cameras">
        <div style="display: flex; width: 100%;">
          <CameraStream style="flex-grow: 1;" :src="url + '/stream?topic=/left/image_raw&quality=30&width=500&height=400'"></CameraStream>
          <CameraStream style="flex-grow: 1;" :src="url + '/stream?topic=/right/image_raw&quality=30&width=500&height=400'"></CameraStream>
          <CameraStream style="flex-grow: 1;" :src="url + '/stream?topic=/disparity_image&quality=30&width=500&height=400'"></CameraStream>
        </div>
      </tab>

      <tab name="Motors">
        <div style="display: flex; width: 100%;">
          <MotorSlider v-for="motor in motors" :key="motor" :address="motor" style="flex-grow: 1;"></MotorSlider>
          <MotorSlider :address="motor" style="flex-grow: 1;"></MotorSlider>
        </div>
      </tab>
    </tabs>

</template>

<script>
import Vue from "vue";
import { Tabs, Tab } from "vue-tabs-component";
import CameraStream from "./camera_stream.vue";
import Motor from "./motor.vue";

Vue.component("tabs", Tabs);
Vue.component("tab", Tab);

Vue.component("CameraStream", CameraStream);
Vue.component("MotorSlider", Motor);

export default {
  props: ["url"],
  data() {
    return {
      motors: [1, 2, 3, 4]
    };
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
