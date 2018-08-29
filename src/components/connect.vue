<template>
    <div class="connect">
        <div class="autocomplete">
          <span>Host:</span>
          <input type="text" list="endpoints" v-model="value"  @focus="focus = true" :disabled="config.connected"/>
          <div v-if="focus">
              <ul class="autocomplete-results">
                  <li class="autocomplete-result" v-for="endpoint in endpoints" :key="endpoint" @click="setResult(endpoint)">{{endpoint}}</li>
              </ul>
          </div>
        </div>

      <button :class="{active: config.connected}" @click="connect()"></button>
  
    </div>
</template>

<script>
export default {
  props: ["endpoints", "config"],
  data() {
    return {
      value: "",
      focus: false
    };
  },
  methods: {
    setResult(v) {
      this.value = v;
      this.focus = false;
    },
    connect() {
      if(!this.config.connected)
        this.config.eventHub.$emit("connect", this.value);
      else
        this.config.eventHub.$emit("disconnect");

      this.focus = false;
    }
  },
  created() {
    if (this.endpoints.length > 0) this.value = this.endpoints[0];
  }
};
</script>

<style lang="css" scoped>
.connect {
  display: flex;
  align-items: center;
}

button {
  width: 6em;
  background-color: #880000;
  border-radius: 50%;
  border: 0;
  padding: 0;
  margin: 1em;
  outline: none;
}

button.active {
  background-color: #008800;
}

button::after {
  content: "";
  display: block;
  padding-bottom: 100%;
}

.autocomplete {
  display: inline-block;
  position: relative;
  margin: 1em;
  width: 15em;
}

span {
  padding: 0.4em;
  position: absolute;
  color: #fff;
}

input {
  width: 100%;
  box-sizing: border-box;
  border: 0;
  background: rgba(0, 0, 0, 50);
  color: #aaa;
  padding: 0.5em;
  padding-left: 3.5em;
  border-radius: 0.25em;
}

.autocomplete-results {
  position: absolute;
  width: 100%;
  bottom: -0.2em;
  transform: translateY(100%);
  padding: 0;
  margin: 0;
  max-height: 5em;
  overflow: auto;
  background-color: rgba(0, 0, 0, 50);
  border-radius: 0.25em;
  color: #fff;
}

.autocomplete-result {
  list-style: none;
  text-align: left;
  padding: 4px 2px;
  cursor: pointer;
}

.autocomplete-result:hover {
  background-color: #0a243f;
}
</style>
