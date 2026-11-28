import { build } from 'esbuild';
import postcss from 'postcss';
import tailwind from 'tailwindcss';
import { readFile, writeFile } from 'node:fs/promises';

await build({
  entryPoints: ['public/studio.js'], bundle: true, minify: true, outfile: 'public/vendor/studio.bundle.js',
  alias: { vue: 'vue/dist/vue.esm-bundler.js' },
  define: { 'process.env.NODE_ENV': '"production"', __VUE_OPTIONS_API__: 'true', __VUE_PROD_DEVTOOLS__: 'false', __VUE_PROD_HYDRATION_MISMATCH_DETAILS__: 'false' }
});
const source = await readFile('workspace.pcss', 'utf8');
const css = await postcss([tailwind({content:['./public/*.html'],corePlugins:{preflight:false}})])
  .process(source,{from:'workspace.pcss'});
await writeFile('public/workspace.css',css.css);
