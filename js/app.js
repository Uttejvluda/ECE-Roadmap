const rolesGrid = document.getElementById("rolesGrid");
const searchInput = document.getElementById("searchInput");
const domainFilter = document.getElementById("domainFilter");

function renderRoles(list) {
  rolesGrid.innerHTML = "";

  if (list.length === 0) {
    rolesGrid.innerHTML = `<p>No roles found ❌</p>`;
    return;
  }

  list.forEach(role => {
    const card = document.createElement("div");
    card.className = "card";
    card.innerHTML = `
      <h3>${role.title}</h3>
      <p>${role.shortDescription}</p>
      <span class="badge">${role.domain}</span>
      <span class="badge">${role.level}</span>
    `;

    card.addEventListener("click", () => {
      window.location.href = `role.html?id=${role.id}`;
    });

    rolesGrid.appendChild(card);
  });
}

function filterRoles() {
  const query = searchInput.value.toLowerCase();
  const domain = domainFilter.value;

  let filtered = rolesData.filter(role => {
    const matchesQuery =
      role.title.toLowerCase().includes(query) ||
      role.shortDescription.toLowerCase().includes(query);

    const matchesDomain = domain === "all" ? true : role.domain === domain;

    return matchesQuery && matchesDomain;
  });

  renderRoles(filtered);
}

searchInput.addEventListener("input", filterRoles);
domainFilter.addEventListener("change", filterRoles);

renderRoles(rolesData);
