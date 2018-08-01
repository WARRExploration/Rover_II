
<template>
  <div style="display: flex; flex-direction: column; min-height: 100vh;">
    <AppHeader title="WARR Exploration - TelOp"></AppHeader>
    <AppBody v-bind="config" style="flex-grow: 1;"></AppBody>

  </div>
</template>

<script>
import Vue from "vue";
import Header from "./components/header.vue";
import Body from "./components/body.vue";

Vue.component("AppHeader", Header);
Vue.component("AppBody", Body);

export default {
  data() {
    return {
      config: {
        url: "http://warr-rover.lrt.mw.tum.de:8080"
        // url: "http://10.0.0.1:8080",
      }
    };
  }
};
</script>

<style lang="css">

/* @font-face {
  font-family: "Made Evolve Sans";
  src: url("./assets/made_evolve_sans/regular.otf") format('opentype');
} */


body {
  font-family: "Made Evolve Sans", Arial, Helvetica, sans-serif;
  margin: 0 auto;
}
</style>
