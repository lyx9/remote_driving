<!--
  FSM-Pilot V2.0 - Remote Driving System

  @project     FSM-Pilot Remote Driving Platform
  @author      Li Yixiang
  @institution City University of Hong Kong
  @copyright   2025 City University of Hong Kong. All rights reserved.
  @license     Proprietary

  @component   App
  @description 应用主入口组件
-->
<template>
  <div class="app-container">
    <router-view />
  </div>
</template>

<script setup lang="ts">
// 使用 Vue Router 进行路由管理
</script>

<style scoped>
.app-container {
  width: 100%;
  height: 100vh;
  overflow: hidden;
}
</style>
