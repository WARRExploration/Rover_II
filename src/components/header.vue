<template>
    <header>
        <img src="../assets/logo.svg">
        <h1>{{ title }}</h1>
    </header>
</template>

<script>
export default {
    props: ['title'],
}
</script>

<style lang="css" scoped>
header {
    display: flex;
    height: 150px;
    background: #333;
    border-bottom: #0A243F solid 0.3em;
}

h1 {
    vertical-align: middle;
    flex-grow: 1;
}
</style>
