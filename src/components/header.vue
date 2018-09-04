<template>
    <header>
        <img src="../assets/logo.svg">
        <h1>{{ title }}</h1>
        <Connect :endpoints="['warr-rover.lrt.mw.tum.de', '10.0.0.1']" :config="config"></Connect>
    </header>
</template>

<script>
import Vue from "vue";
import connect from "./connect.vue";

Vue.component("Connect", connect);

export default {
  props: ["title", "config"]
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
</style>
