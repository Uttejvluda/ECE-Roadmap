const params = new URLSearchParams(window.location.search);
const roleId = params.get("id");

const roleTitle = document.getElementById("roleTitle");
const roleSubtitle = document.getElementById("roleSubtitle");
const roleDomain = document.getElementById("roleDomain");
const roleSalary = document.getElementById("roleSalary");
const roleCompanies = document.getElementById("roleCompanies");
const tabContent = document.getElementById("tabContent");

const tabButtons = document.querySelectorAll(".tabBtn");

const role = rolesData.find(r => r.id === roleId);

if (!role) {
  roleTitle.innerText = "Role not found ❌";
  roleSubtitle.innerText = "Invalid role ID.";
} else {
  roleTitle.innerText = role.title;
  roleSubtitle.innerText = role.shortDescription;
  roleDomain.innerText = `Domain: ${role.domain}`;
  roleSalary.innerText = `Salary: ${role.salary}`;
  roleCompanies.innerText = `Companies: ${role.companies}`;
}

function createList(items) {
  return `<ul>${items.map(i => `<li>${i}</li>`).join("")}</ul>`;
}

function renderStage(stageKey) {
  const stage = role.roadmap[stageKey];

  tabContent.innerHTML = `
    <div class="sectionBox">
      <h3>📌 Fundamentals</h3>
      ${createList(stage.fundamentals)}
    </div>

    <div class="sectionBox">
      <h3>🧠 Skills to Learn</h3>
      ${createList(stage.skills)}
    </div>

    <div class="sectionBox">
      <h3>🛠 Tools</h3>
      ${createList(stage.tools)}
    </div>

    <div class="sectionBox">
      <h3>🚀 Projects</h3>
      ${createList(stage.projects)}
    </div>

    <div class="sectionBox">
      <h3>⭐ Extra / Others</h3>
      ${createList(stage.others)}
    </div>
  `;
}

tabButtons.forEach(btn => {
  btn.addEventListener("click", () => {
    tabButtons.forEach(b => b.classList.remove("active"));
    btn.classList.add("active");
    renderStage(btn.dataset.tab);
  });
});

if (role) renderStage("basic");
