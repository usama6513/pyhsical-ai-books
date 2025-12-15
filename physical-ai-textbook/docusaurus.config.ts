const math = require('remark-math');
const katex = require('rehype-katex');

module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Digital Brain to Physical Body',
  url: 'https://usama6513.github.io',
  baseUrl: '/pyhsical-ai-books/',
  organizationName: 'usama6513',
  projectName: 'pyhsical-ai-books',
  onBrokenLinks: 'warn',
  
  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          remarkPlugins: [math],
          rehypePlugins: [katex],
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'Physical AI Book',
      items: [
        {
          type: 'doc',
          docId: 'intro',
          position: 'left',
          label: 'Read Book',
        },
      ],
    },
    prism: {
      additionalLanguages: ['python', 'bash'],
    },
  },
  
  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],
  
  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.13.24/dist/katex.min.css',
      type: 'text/css',
      integrity: 'sha384-odtC+0UGzzFL/6PNoE8rX/SPcQDXBJ+uRepguP4QkPCm2LBxH3FA3y+fKSiJ+AmM',
      crossorigin: 'anonymous',
    },
  ],
};
