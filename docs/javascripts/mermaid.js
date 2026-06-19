document.addEventListener("DOMContentLoaded", () => {
  mermaid.initialize({
    startOnLoad: true,
    theme: window.matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "default",
  });
});
