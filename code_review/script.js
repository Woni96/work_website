const filterButtons = document.querySelectorAll(".chip-button");
const filterTargets = document.querySelectorAll("[data-category]");
const tocLinks = document.querySelectorAll(".toc a");
const sections = document.querySelectorAll("main section[id]");
const tabs = document.querySelectorAll(".tab");
const tabPanels = document.querySelectorAll(".tab-panel");

filterButtons.forEach((button) => {
  button.addEventListener("click", () => {
    const filter = button.dataset.filter;

    filterButtons.forEach((item) => item.classList.remove("is-active"));
    button.classList.add("is-active");

    filterTargets.forEach((section) => {
      if (filter === "all") {
        section.dataset.hidden = "false";
        return;
      }

      const categories = section.dataset.category.split(" ");
      section.dataset.hidden = categories.includes(filter) ? "false" : "true";
    });
  });
});

tabs.forEach((tab) => {
  tab.addEventListener("click", () => {
    const targetId = tab.dataset.tabTarget;

    tabs.forEach((item) => item.classList.remove("is-active"));
    tabPanels.forEach((panel) => panel.classList.remove("is-active"));

    tab.classList.add("is-active");
    document.getElementById(targetId)?.classList.add("is-active");
  });
});

const observer = new IntersectionObserver(
  (entries) => {
    entries.forEach((entry) => {
      if (!entry.isIntersecting) {
        return;
      }

      const currentId = entry.target.id;
      tocLinks.forEach((link) => {
        link.classList.toggle("is-active", link.getAttribute("href") === `#${currentId}`);
      });
    });
  },
  {
    rootMargin: "-30% 0px -55% 0px",
    threshold: 0.1,
  }
);

sections.forEach((section) => observer.observe(section));
