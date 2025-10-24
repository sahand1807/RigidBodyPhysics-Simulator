# API Documentation Hosting Setup

This project uses **GitHub Actions** to automatically build and deploy API documentation to **GitHub Pages**.

## What Gets Built

1. **C++ API Documentation** (Doxygen)
   - Class hierarchies and inheritance diagrams
   - Call graphs and collaboration diagrams
   - Complete API reference for all classes

2. **Python API Documentation** (Sphinx)
   - Python bindings and visualization modules
   - Type-hinted API reference
   - Module documentation

## Automatic Deployment

The documentation is automatically built and deployed when you:
- Push to `master` or `main` branch
- Manually trigger the workflow from GitHub Actions tab

**No generated files are committed to the repository** - everything is built in the cloud.

## Setup Instructions

### 1. Enable GitHub Pages

1. Go to your repository on GitHub
2. Click **Settings** → **Pages** (in left sidebar)
3. Under "Build and deployment":
   - **Source**: Select "GitHub Actions"
4. Save the settings

### 2. Push the Workflow

```bash
git add .github/workflows/docs.yml
git add docs/README_HOSTING.md
git add .gitignore
git commit -m "Add GitHub Actions workflow for API documentation"
git push
```

### 3. Monitor the Build

1. Go to **Actions** tab in your GitHub repository
2. You'll see "Build and Deploy API Documentation" workflow running
3. Wait for it to complete (takes ~2-3 minutes)

### 4. Access Your Documentation

Once deployed, your documentation will be available at:

```
https://<your-github-username>.github.io/<repository-name>/
```

- **C++ API**: `https://<your-username>.github.io/<repo-name>/cpp/`
- **Python API**: `https://<your-username>.github.io/<repo-name>/python/`

## Local Testing

To test documentation builds locally before pushing:

### Test C++ Documentation
```bash
# Generate Doxygen docs
doxygen Doxyfile

# View locally
open docs/api/cpp/html/index.html
```

### Test Python Documentation
```bash
# Generate Sphinx docs
cd python
sphinx-apidoc -f -o docs/api/source physics_viz
cd docs/api
make html

# View locally
open build/html/index.html
```

## Troubleshooting

### Workflow Fails on First Run

**Problem**: GitHub Pages not enabled or wrong source selected

**Solution**:
1. Go to Settings → Pages
2. Set Source to "GitHub Actions" (not "Deploy from a branch")

### 404 Error After Deployment

**Problem**: Site not ready yet or wrong URL

**Solution**:
- Wait 1-2 minutes after deployment
- Check the URL in Settings → Pages
- Ensure workflow completed successfully in Actions tab

### Documentation Looks Empty

**Problem**: Source files not included in build

**Solution**:
- Ensure all source RST files are committed
- Check workflow logs in Actions tab for build errors

## Updating Documentation

Documentation automatically updates when you:
- Modify C++ code with Doxygen comments
- Update Python docstrings
- Change Sphinx RST files
- Push to master/main branch

No manual intervention needed!

## Manual Trigger

You can manually trigger documentation rebuild without code changes:

1. Go to **Actions** tab
2. Select "Build and Deploy API Documentation"
3. Click **Run workflow** button
4. Select branch and click green "Run workflow" button

## What's In Git vs What's Generated

### Committed to Git (Source Files)
- `Doxyfile` - Doxygen configuration
- `python/docs/api/source/conf.py` - Sphinx configuration
- `python/docs/api/source/*.rst` - Documentation source files
- `.github/workflows/docs.yml` - Build workflow

### Generated (NOT in Git)
- `docs/api/cpp/` - Doxygen HTML output
- `python/docs/api/build/` - Sphinx HTML output

These are excluded via `.gitignore` and only exist:
- Locally after running build commands
- In GitHub Actions during workflow
- On GitHub Pages branch (gh-pages)

## Cost

**FREE** - GitHub Pages and GitHub Actions are free for public repositories.
