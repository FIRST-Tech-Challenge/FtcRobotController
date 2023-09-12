// This script should  move all local branches with no unmerged changes forward
// to the latest 'main', and then push them

const { readFile } = require('fs/promises');
const { cwd } = require('process');
const { simpleGit } = require('simple-git');

const git = simpleGit();

let status = null;

// Read a bunch of branch names from 'scripts/branch-list.txt' and return them as a Set
async function createBranchList(branches) {
  try {
    // Check to see if we have a branch-list.txt file already
    const contents = await readFile('scripts/branch-list.txt', 'utf8');
    const resultArray = contents.split(/\n|\r/);
    return new Set(
      resultArray
        .map((str) => str.trim())
        .filter((str) => str.length > 0 && str[0] !== '#'),
    );
  } catch (e) {
    // Some file access problem: Let's look for local branches
    console.error(
      'Cowardly refusing to do repo things without some valid input.',
    );
    console.error('Here are the current local branches in your repo:');
    Object.entries(branches).forEach(([name]) => console.error('  ', name));
    console.error(
      'You should put some of them in the scripts/branch-list.txt file.',
    );
    console.error('Only then will this script acutally do anything...');
  }
  return new Set();
}

const defBranch = 'main';

async function fastforward() {
  // Make sure whatever is currently checked out is clean...
  status = await git.status();
  if (!status.isClean()) {
    throw Error("Current branch doesn't appear to be clean- Aborting!");
  }
  // Check out default branch
  console.log(await git.checkout(defBranch));
  // Pull from origin
  await git.pull();
  // Okay, main is up-to-date, let's get our list of local branches
  const branches = (await git.branchLocal()).branches;
  const mainCommit = branches[defBranch].commit;
  // For each branch, check to see if it should be fast-forwarded
  const list = await createBranchList(branches);
  if (list.size === 0) {
    console.error('No branches found');
    return;
  }
  for (let [, branch] of Object.entries(branches)) {
    if (branch.name === defBranch) {
      continue;
    }
    // Filter this to only branches in a branch-list.txt file
    // with the option to create it from current local branches
    if (!list.has(branch.name)) {
      console.log(
        'Branch not in your scripts/branch-list.txt file:',
        branch.name,
      );
      continue;
    }
    if (branch.commit === mainCommit) {
      console.log(
        'Branch is already up-to-date (locally) skipping:',
        branch.name,
      );
      continue;
    }
    console.log(`Checking out ${branch.name}`);
    await git.checkout(branch.name);
    // First, let's try to make sure we've pulled
    await git.pull();
    // Now let's try to fast-forward this branch
    const res = await git.merge(['--ff-only', defBranch]);
    if (res.failed) {
      console.error(`Branch ${branch.name} failed to merge!`);
    } else {
      // It was fast-forwarded: Push it!
      await git.push('origin');
    }
  }
}

// Call our script...
fastforward()
  // Reporting errors
  .catch((err) => console.error(err))
  // and restoring our original branch, if possible
  .finally(() => (status != null ? git.checkout(status.current) : undefined));
