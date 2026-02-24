<!--
  FSM-Pilot V2.0 - Remote Driving System

  @project     FSM-Pilot Remote Driving Platform
  @author      Li Yixiang
  @institution City University of Hong Kong
  @copyright   2025 City University of Hong Kong. All rights reserved.
  @license     Proprietary

  @component   RosBagDemo
  @description RosBag 回放功能演示页面
-->
<template>
  <div class="rosbag-demo">
    <RosBagReplayPro />
  </div>
</template>

<script setup lang="ts">
import RosBagReplayPro from './RosBagReplayPro.vue'
</script>

<style scoped>
.rosbag-demo {
  width: 100vw;
  height: 100vh;
  overflow: hidden;
}
</style>
