import yaml
import os


yamldir = "./yaml/"

### Find all files in yaml subdirectory
teams = sorted(os.listdir(yamldir))
data = {}

for team in teams:
    ### Try to open each one
    with open(yamldir + team, 'r') as stream:
        try:
            ### Load yaml data
            dat = yaml.safe_load(stream)
            ### Save indexed by team name
            data[team[:-5].capitalize()] = dat

        except yaml.YAMLError as exc:
            raise exc


for teamname, yamldata in data.items():
    print("Team " + teamname)

    # Ensure all the weeks are there
    weeks = [x["week"] for x in yamldata]
    assert(weeks == list(range(1,11)))

    # Extract then print each week's milestones in order
    for w in yamldata:
        total = 0
        print ("Week %d:" % w["week"])
        for g in w["goals"]:
            total += g["points"]
            print ("  Goal (%d/10) %s" % (g["points"], g["goal"]))
            print ("  Deliverable: %s" % g["deliverable"])
            for t in g["tasks"]:
                print ("    - %s" % t)
        assert(total == 10)


print ("***")
print ("*** Success!")
print ("***")
