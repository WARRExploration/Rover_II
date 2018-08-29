
<template>
    <div style="display: flex; flex-direction: column; min-height: 100vh;">
        <AppHeader v-if="!config.fullscreen" :config="config" title="WARR Exploration - TelOp" ></AppHeader>
        <AppBody :config="config" style="flex-grow: 1;"></AppBody>
      </div>
</template>

<script>
import Vue from "vue";
import Header from "./components/header.vue";
import Body from "./components/body.vue";

Vue.component("AppHeader", Header);
Vue.component("AppBody", Body);
var eventHub = new Vue();

export default {
  data() {
    return {
      config: {
        ros: null,
        connected: false,
        eventHub: eventHub,
        fullscreen: false,
        url: '',
      }
    };
  },
  methods: {
    connect(url) {
      this.config.ros = new ROSLIB.Ros({
        url: "ws://" + url + ":9090"
      });

      this.config.ros.on(
        "connection",
        function() {
          this.config.connected = true;
          this.config.url = url;
        }.bind(this)
      );

      this.config.ros.on(
        "close",
        function() {
          this.config.connected = false;
          this.config.url = '';
        }.bind(this)
      );
    },
    disconnect() {
      this.config.ros.close();
    }
  },
  created() {
    this.config.eventHub.$on("connect", this.connect);
    this.config.eventHub.$on("disconnect", this.disconnect);

    document.addEventListener(
      "keydown",
      function(zEvent) {
        if (zEvent.ctrlKey && zEvent.shiftKey && zEvent.code === "KeyF") {
          this.config.fullscreen = !this.config.fullscreen;

          if (this.config.fullscreen) {
            var el = document.documentElement,
              rfs =
                el.requestFullscreen ||
                el.webkitRequestFullScreen ||
                el.mozRequestFullScreen ||
                el.msRequestFullscreen;

            rfs.call(el);
          } else {
            if (document.exitFullscreen) document.exitFullscreen();
            else if (document.msExitFullscreen) document.msExitFullscreen();
            else if (document.mozCancelFullScreen)
              document.mozCancelFullScreen();
            else if (document.webkitExitFullscreen)
              document.webkitExitFullscreen();
          }
        }
      }.bind(this)
    );
  }
};
</script>

<style lang="css">
@font-face {
  font-family: "Made Evolve Sans";
  src: url("./assets/made_evolve_sans/regular.eot");
  src: url("./assets/made_evolve_sans/regular.eot?#iefix")
      format("embedded-opentype"),
    url("./assets/made_evolve_sans/regular.woff") format("woff"),
    url("./assets/made_evolve_sans/regular.ttf") format("truetype");
  font-weight: 400;
  font-style: normal;
}

body {
  margin: 0 auto;
}
body,
input {
  font-family: "Made Evolve Sans", Arial, Helvetica, sans-serif;
}
</style>
