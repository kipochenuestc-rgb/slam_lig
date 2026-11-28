const openLocalization = new URLSearchParams(location.search).get('tab') === 'localization';
import {createApp} from 'vue';
import Button from 'ant-design-vue/es/button';
import { RadarChartOutlined, EnvironmentOutlined, DotChartOutlined, AimOutlined, ExportOutlined } from '@ant-design/icons-vue';
createApp({
  data: () => ({page:openLocalization ? 'localization' : 'mapping', localizationOpened:openLocalization, pointcloudOpened:false, atlasOpened:false, selected:'', loaded:'', datasets:[], notice:'', showDirectoryInput:false, folderPath:''}),
  computed: {
    atlasUrl() {
      return this.atlasOpened && this.loaded ? '/atlas/' + encodeURIComponent(this.loaded) + '/index.html?view=satellite' : '';
    },
    pointcloudUrl() {
      return this.pointcloudOpened ? '/pointcloud.html?dataset=' + encodeURIComponent(this.loaded) : '';
    }
  },
  methods: {
    async openVectorEditor(event) {
      const api = window.pywebview?.api;
      if (typeof api?.open_vector_editor !== 'function') return;
      event.preventDefault();
      try {
        if (!await api.open_vector_editor()) throw Error('系统浏览器未能打开地图编辑器');
      } catch (error) { this.notice = error.message; }
    },
    goMapping() { this.page = 'mapping'; this.notice = ''; },
    async goAtlas(current) {
      this.page = 'atlas';
      this.atlasOpened = true;
      this.notice = '';
      try {
        await this.refreshDatasets();
        if (current) {
          const job = await (await fetch('/api/mapping')).json();
          const id = job.runDir?.split(/[\\/]/).pop();
          const candidate = id;
          if (this.datasets.some(item => item.id === candidate)) {
            this.selected = candidate;
            this.loadSelected();
            return;
          }
          this.notice = '当前任务尚无可投影的地理对齐地图，可在上方选择已有地图。';
        }
        if (!this.datasets.some(item => item.id === this.selected)) this.selected = this.datasets[0]?.id || '';
        if (!this.selected) this.notice = '还没有可显示的地图。';
        else if (!this.loaded && !this.notice) this.notice = '选择地图后点击“载入地图”，即可在高德卫星影像上查看。';
      } catch (error) { this.notice = error.message; }
    },
    async goPointcloud() {
      this.page = 'pointcloud';
      this.pointcloudOpened = true;
      this.notice = '';
      try {
        await this.refreshDatasets();
        if (!this.selected) {
          this.notice = '还没有可显示的地图。';
          return;
        }
        if (!this.loaded) this.loaded = this.selected;
      } catch (error) { this.notice = error.message; }
    },
    async refreshDatasets() {
      const response = await fetch('/api/datasets');
      if (!response.ok) throw Error('读取地图列表失败');
      this.datasets = await response.json();
      if (!this.datasets.some(item => item.id === this.selected)) this.selected = this.datasets[0]?.id || '';
    },
    async browseMap() {
      try {
        const picker = window.pywebview?.api;
        if (typeof picker?.choose_directory !== 'function') {
          this.showDirectoryInput = true;
          this.notice = '';
          return;
        }
        const directory = await picker.choose_directory();
        if (!directory) return;
        await this.importMapFolder(directory);
      } catch (error) { this.notice = error.message; }
    },
    async importMapFolder(directory=this.folderPath.trim()) {
      if (!directory) {
        this.notice = '请输入地图文件夹的完整路径。';
        return;
      }
      try {
        this.notice = '正在读取地图…';
        const response = await fetch('/api/datasets/import', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({directory})});
        const result = await response.json();
        if (!response.ok) throw Error(result.detail || '读取地图失败');
        await this.refreshDatasets();
        this.selected = result.id;
        this.showDirectoryInput = false;
        this.folderPath = '';
        this.loadSelected();
      } catch (error) { this.notice = error.message; }
    },
    cancelDirectoryInput() {
      this.showDirectoryInput = false;
      this.folderPath = '';
      this.notice = '';
    },
    loadSelected() {
      if (!this.selected) return;
      this.loaded = this.selected;
      this.notice = '';
    },
    labelFor(id) { return this.datasets.find(item => item.id === id)?.label || id; },
    wireMapping(event) {
      const link = event.target.contentDocument.querySelector('.mapping-heading a');
      if (link) {
        link.textContent = '在高德上查看建图结果 →';
        link.onclick = e => { e.preventDefault(); this.goAtlas(true); };
      }
    },
    wireAtlas(event) {
      const doc = event.target.contentDocument;
      for (const link of doc.querySelectorAll('.brand, .page-link')) {
        link.onclick = e => { e.preventDefault(); this.goMapping(); };
      }
      if (this.loaded.startsWith('web_')) {
        doc.querySelector('.session h2').textContent = '离线建图结果';
        doc.querySelector('.session p').textContent = this.loaded;
      }
    }
  }
}).component('AButton',Button).component('RadarChartOutlined',RadarChartOutlined)
  .component('EnvironmentOutlined',EnvironmentOutlined).component('DotChartOutlined',DotChartOutlined)
  .component('AimOutlined',AimOutlined).component('ExportOutlined',ExportOutlined).mount('#studio');
