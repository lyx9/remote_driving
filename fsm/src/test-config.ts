// 测试环境变量是否正确加载
console.log('=== Amap Configuration Test ===');
console.log('VITE_AMAP_API_KEY:', import.meta.env.VITE_AMAP_API_KEY);
console.log('VITE_AMAP_JS_CODE:', import.meta.env.VITE_AMAP_JS_CODE);

import { getAPIConfig, isAPIConfigured } from './config/apiConfig';

const config = getAPIConfig();
console.log('API Config:', config.amap);
console.log('Is Amap Configured:', isAPIConfigured('amap'));
console.log('================================');
