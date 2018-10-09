<template>
    <header>
        <img src="../assets/logo.svg">
        <h1>{{ title }}</h1>
        <div v-if="config.connected">
          <button @click="setRed" :class="{active: red}">Red</button>
          <button @click="setGreen" :class="{active: green}">Green</button>
        </div>
        <Connect :endpoints="['warr-rover.lrt.mw.tum.de', '10.0.0.1', 'localhost']" :config="config"></Connect>
    </header>
</template>

<script>
import Vue from "vue";
import connect from "./connect.vue";

Vue.component("Connect", connect);

export default {
  props: ["title", "config"],
  data() {
    return {
      red: true,
      green: false
    };
  },
  methods: {
    setRed() {
      this.red = !this.red;
      var topic = new ROSLIB.Topic({
        ros: this.config.ros,
        name: "/lamp/r",
        messageType: "std_msg/Bool"
      });

      var msg = new ROSLIB.Message({
        data: this.red
      });
      topic.publish(msg);
    },
    setGreen() {
      this.green = !this.green;
      var topic = new ROSLIB.Topic({
        ros: this.config.ros,
        name: "/lamp/g",
        messageType: "std_msg/Bool"
      });

      var msg = new ROSLIB.Message({
        data: this.green
      });
      topic.publish(msg);
    }
  }
};
</script>

<style lang="css" scoped>
header {
  display: flex;
  height: 15vh;
  background: #333;
  border-bottom: #0a243f solid 0.3em;
  align-items: center;
}

h1 {
  flex-grow: 1;
  color: #fff;
}

img {
  height: 100%;
}

button.active {
  background-color: #88ff88;
}
</style>
