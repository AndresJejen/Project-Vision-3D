/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2013-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * person_cluster.hpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */

#ifndef PCL_PEOPLE_PERSON_CLUSTER_HPP_
#define PCL_PEOPLE_PERSON_CLUSTER_HPP_

#include <pcl/people/person_cluster.h>

template <typename PointT>
pcl::people::PersonCluster<PointT>::PersonCluster (
    const PointCloudPtr& input_cloud,
    const pcl::PointIndices& indices,
    const Eigen::VectorXf& ground_coeffs,
    float sqrt_ground_coeffs,
    bool head_centroid,
    bool vertical)
    {
      init(input_cloud, indices, ground_coeffs, sqrt_ground_coeffs, head_centroid, vertical);
      object_confidence_ = {0,0};
      classList_ = {"tench",
					"goldfish",
					"great white shark",
					"tiger shark",
					"hammerhead",
					"electric ray",
					"stingray",
					"cock",
					"hen",
					"ostrich",
					"brambling",
					"goldfinch",
					"house finch",
					"junco",
					"indigo bunting",
					"robin",
					"bulbul",
					"jay",
					"magpie",
					"chickadee",
					"water ouzel",
					"kite",
					"bald eagle",
					"vulture",
					"great grey owl",
					"European fire salamander",
					"common newt",
					"eft",
					"spotted salamander",
					"axolotl",
					"bullfrog",
					"tree frog",
					"tailed frog",
					"loggerhead",
					"leatherback turtle",
					"mud turtle",
					"terrapin",
					"box turtle",
					"banded gecko",
					"common iguana",
					"American chameleon",
					"whiptail",
					"agama",
					"frilled lizard",
					"alligator lizard",
					"Gila monster",
					"green lizard",
					"African chameleon",
					"Komodo dragon",
					"African crocodile",
					"American alligator",
					"triceratops",
					"thunder snake",
					"ringneck snake",
					"hognose snake",
					"green snake",
					"king snake",
					"garter snake",
					"water snake",
					"vine snake",
					"night snake",
					"boa constrictor",
					"rock python",
					"Indian cobra",
					"green mamba",
					"sea snake",
					"horned viper",
					"diamondback",
					"sidewinder",
					"trilobite",
					"harvestman",
					"scorpion",
					"black and gold garden spider",
					"barn spider",
					"garden spider",
					"black widow",
					"tarantula",
					"wolf spider",
					"tick",
					"centipede",
					"black grouse",
					"ptarmigan",
					"ruffed grouse",
					"prairie chicken",
					"peacock",
					"quail",
					"partridge",
					"African grey",
					"macaw",
					"sulphur-crested cockatoo",
					"lorikeet",
					"coucal",
					"bee eater",
					"hornbill",
					"hummingbird",
					"jacamar",
					"toucan",
					"drake",
					"red-breasted merganser",
					"goose",
					"black swan",
					"tusker",
					"echidna",
					"platypus",
					"wallaby",
					"koala",
					"wombat",
					"jellyfish",
					"sea anemone",
					"brain coral",
					"flatworm",
					"nematode",
					"conch",
					"snail",
					"slug",
					"sea slug",
					"chiton",
					"chambered nautilus",
					"Dungeness crab",
					"rock crab",
					"fiddler crab",
					"king crab",
					"American lobster",
					"spiny lobster",
					"crayfish",
					"hermit crab",
					"isopod",
					"white stork",
					"black stork",
					"spoonbill",
					"flamingo",
					"little blue heron",
					"American egret",
					"bittern",
					"crane",
					"limpkin",
					"European gallinule",
					"American coot",
					"bustard",
					"ruddy turnstone",
					"red-backed sandpiper",
					"redshank",
					"dowitcher",
					"oystercatcher",
					"pelican",
					"king penguin",
					"albatross",
					"grey whale",
					"killer whale",
					"dugong",
					"sea lion",
					"Chihuahua",
					"Japanese spaniel",
					"Maltese dog",
					"Pekinese",
					"Shih-Tzu",
					"Blenheim spaniel",
					"papillon",
					"toy terrier",
					"Rhodesian ridgeback",
					"Afghan hound",
					"basset",
					"beagle",
					"bloodhound",
					"bluetick",
					"black-and-tan coonhound",
					"Walker hound",
					"English foxhound",
					"redbone",
					"borzoi",
					"Irish wolfhound",
					"Italian greyhound",
					"whippet",
					"Ibizan hound",
					"Norwegian elkhound",
					"otterhound",
					"Saluki",
					"Scottish deerhound",
					"Weimaraner",
					"Staffordshire bullterrier",
					"American Staffordshire terrier",
					"Bedlington terrier",
					"Border terrier",
					"Kerry blue terrier",
					"Irish terrier",
					"Norfolk terrier",
					"Norwich terrier",
					"Yorkshire terrier",
					"wire-haired fox terrier",
					"Lakeland terrier",
					"Sealyham terrier",
					"Airedale",
					"cairn",
					"Australian terrier",
					"Dandie Dinmont",
					"Boston bull",
					"miniature schnauzer",
					"giant schnauzer",
					"standard schnauzer",
					"Scotch terrier",
					"Tibetan terrier",
					"silky terrier",
					"soft-coated wheaten terrier",
					"West Highland white terrier",
					"Lhasa",
					"flat-coated retriever",
					"curly-coated retriever",
					"golden retriever",
					"Labrador retriever",
					"Chesapeake Bay retriever",
					"German short-haired pointer",
					"vizsla",
					"English setter",
					"Irish setter",
					"Gordon setter",
					"Brittany spaniel",
					"clumber",
					"English springer",
					"Welsh springer spaniel",
					"cocker spaniel",
					"Sussex spaniel",
					"Irish water spaniel",
					"kuvasz",
					"schipperke",
					"groenendael",
					"malinois",
					"briard",
					"kelpie",
					"komondor",
					"Old English sheepdog",
					"Shetland sheepdog",
					"collie",
					"Border collie",
					"Bouvier des Flandres",
					"Rottweiler",
					"German shepherd",
					"Doberman",
					"miniature pinscher",
					"Greater Swiss Mountain dog",
					"Bernese mountain dog",
					"Appenzeller",
					"EntleBucher",
					"boxer",
					"bull mastiff",
					"Tibetan mastiff",
					"French bulldog",
					"Great Dane",
					"Saint Bernard",
					"Eskimo dog",
					"malamute",
					"Siberian husky",
					"dalmatian",
					"affenpinscher",
					"basenji",
					"pug",
					"Leonberg",
					"Newfoundland",
					"Great Pyrenees",
					"Samoyed",
					"Pomeranian",
					"chow",
					"keeshond",
					"Brabancon griffon",
					"Pembroke",
					"Cardigan",
					"toy poodle",
					"miniature poodle",
					"standard poodle",
					"Mexican hairless",
					"timber wolf",
					"white wolf",
					"red wolf",
					"coyote",
					"dingo",
					"dhole",
					"African hunting dog",
					"hyena",
					"red fox",
					"kit fox",
					"Arctic fox",
					"grey fox",
					"tabby",
					"tiger cat",
					"Persian cat",
					"Siamese cat",
					"Egyptian cat",
					"cougar",
					"lynx",
					"leopard",
					"snow leopard",
					"jaguar",
					"lion",
					"tiger",
					"cheetah",
					"brown bear",
					"American black bear",
					"ice bear",
					"sloth bear",
					"mongoose",
					"meerkat",
					"tiger beetle",
					"ladybug",
					"ground beetle",
					"long-horned beetle",
					"leaf beetle",
					"dung beetle",
					"rhinoceros beetle",
					"weevil",
					"fly",
					"bee",
					"ant",
					"grasshopper",
					"cricket",
					"walking stick",
					"cockroach",
					"mantis",
					"cicada",
					"leafhopper",
					"lacewing",
					"dragonfly",
					"damselfly",
					"admiral",
					"ringlet",
					"monarch",
					"cabbage butterfly",
					"sulphur butterfly",
					"lycaenid",
					"starfish",
					"sea urchin",
					"sea cucumber",
					"wood rabbit",
					"hare",
					"Angora",
					"hamster",
					"porcupine",
					"fox squirrel",
					"marmot",
					"beaver",
					"guinea pig",
					"sorrel",
					"zebra",
					"hog",
					"wild boar",
					"warthog",
					"hippopotamus",
					"ox",
					"water buffalo",
					"bison",
					"ram",
					"bighorn",
					"ibex",
					"hartebeest",
					"impala",
					"gazelle",
					"Arabian camel",
					"llama",
					"weasel",
					"mink",
					"polecat",
					"black-footed ferret",
					"otter",
					"skunk",
					"badger",
					"armadillo",
					"three-toed sloth",
					"orangutan",
					"gorilla",
					"chimpanzee",
					"gibbon",
					"siamang",
					"guenon",
					"patas",
					"baboon",
					"macaque",
					"langur",
					"colobus",
					"proboscis monkey",
					"marmoset",
					"capuchin",
					"howler monkey",
					"titi",
					"spider monkey",
					"squirrel monkey",
					"Madagascar cat",
					"indri",
					"Indian elephant",
					"African elephant",
					"lesser panda",
					"giant panda",
					"barracouta",
					"eel",
					"coho",
					"rock beauty",
					"anemone fish",
					"sturgeon",
					"gar",
					"lionfish",
					"puffer",
					"abacus",
					"abaya",
					"academic gown",
					"accordion",
					"acoustic guitar",
					"aircraft carrier",
					"airliner",
					"airship",
					"altar",
					"ambulance",
					"amphibian",
					"analog clock",
					"apiary",
					"apron",
					"ashcan",
					"assault rifle",
					"backpack",
					"bakery",
					"balance beam",
					"balloon",
					"ballpoint",
					"Band Aid",
					"banjo",
					"bannister",
					"barbell",
					"barber chair",
					"barbershop",
					"barn",
					"barometer",
					"barrel",
					"barrow",
					"baseball",
					"basketball",
					"bassinet",
					"bassoon",
					"bathing cap",
					"bath towel",
					"bathtub",
					"beach wagon",
					"beacon",
					"beaker",
					"bearskin",
					"beer bottle",
					"beer glass",
					"bell cote",
					"bib",
					"bicycle-built-for-two",
					"bikini",
					"binder",
					"binoculars",
					"birdhouse",
					"boathouse",
					"bobsled",
					"bolo tie",
					"bonnet",
					"bookcase",
					"bookshop",
					"bottlecap",
					"bow",
					"bow tie",
					"brass",
					"brassiere",
					"breakwater",
					"breastplate",
					"broom",
					"bucket",
					"buckle",
					"bulletproof vest",
					"bullet train",
					"butcher shop",
					"cab",
					"caldron",
					"candle",
					"cannon",
					"canoe",
					"can opener",
					"cardigan",
					"car mirror",
					"carousel",
					"carpenter's kit",
					"carton",
					"car wheel",
					"cash machine",
					"cassette",
					"cassette player",
					"castle",
					"catamaran",
					"CD player",
					"cello",
					"cellular telephone",
					"chain",
					"chainlink fence",
					"chain mail",
					"chain saw",
					"chest",
					"chiffonier",
					"chime",
					"china cabinet",
					"Christmas stocking",
					"church",
					"cinema",
					"cleaver",
					"cliff dwelling",
					"cloak",
					"clog",
					"cocktail shaker",
					"coffee mug",
					"coffeepot",
					"coil",
					"combination lock",
					"computer keyboard",
					"confectionery",
					"container ship",
					"convertible",
					"corkscrew",
					"cornet",
					"cowboy boot",
					"cowboy hat",
					"cradle",
					"crane",
					"crash helmet",
					"crate",
					"crib",
					"Crock Pot",
					"croquet ball",
					"crutch",
					"cuirass",
					"dam",
					"desk",
					"desktop computer",
					"dial telephone",
					"diaper",
					"digital clock",
					"digital watch",
					"dining table",
					"dishrag",
					"dishwasher",
					"disk brake",
					"dock",
					"dogsled",
					"dome",
					"doormat",
					"drilling platform",
					"drum",
					"drumstick",
					"dumbbell",
					"Dutch oven",
					"electric fan",
					"electric guitar",
					"electric locomotive",
					"entertainment center",
					"envelope",
					"espresso maker",
					"face powder",
					"feather boa",
					"file",
					"fireboat",
					"fire engine",
					"fire screen",
					"flagpole",
					"flute",
					"folding chair",
					"football helmet",
					"forklift",
					"fountain",
					"fountain pen",
					"four-poster",
					"freight car",
					"French horn",
					"frying pan",
					"fur coat",
					"garbage truck",
					"gasmask",
					"gas pump",
					"goblet",
					"go-kart",
					"golf ball",
					"golfcart",
					"gondola",
					"gong",
					"gown",
					"grand piano",
					"greenhouse",
					"grille",
					"grocery store",
					"guillotine",
					"hair slide",
					"hair spray",
					"half track",
					"hammer",
					"hamper",
					"hand blower",
					"hand-held computer",
					"handkerchief",
					"hard disc",
					"harmonica",
					"harp",
					"harvester",
					"hatchet",
					"holster",
					"home theater",
					"honeycomb",
					"hook",
					"hoopskirt",
					"horizontal bar",
					"horse cart",
					"hourglass",
					"iPod",
					"iron",
					"jack-o'-lantern",
					"jean",
					"jeep",
					"jersey",
					"jigsaw puzzle",
					"jinrikisha",
					"joystick",
					"kimono",
					"knee pad",
					"knot",
					"lab coat",
					"ladle",
					"lampshade",
					"laptop",
					"lawn mower",
					"lens cap",
					"letter opener",
					"library",
					"lifeboat",
					"lighter",
					"limousine",
					"liner",
					"lipstick",
					"Loafer",
					"lotion",
					"loudspeaker",
					"loupe",
					"lumbermill",
					"magnetic compass",
					"mailbag",
					"mailbox",
					"maillot",
					"maillot",
					"manhole cover",
					"maraca",
					"marimba",
					"mask",
					"matchstick",
					"maypole",
					"maze",
					"measuring cup",
					"medicine chest",
					"megalith",
					"microphone",
					"microwave",
					"military uniform",
					"milk can",
					"minibus",
					"miniskirt",
					"minivan",
					"missile",
					"mitten",
					"mixing bowl",
					"mobile home",
					"Model T",
					"modem",
					"monastery",
					"monitor",
					"moped",
					"mortar",
					"mortarboard",
					"mosque",
					"mosquito net",
					"motor scooter",
					"mountain bike",
					"mountain tent",
					"mouse",
					"mousetrap",
					"moving van",
					"muzzle",
					"nail",
					"neck brace",
					"necklace",
					"nipple",
					"notebook",
					"obelisk",
					"oboe",
					"ocarina",
					"odometer",
					"oil filter",
					"organ",
					"oscilloscope",
					"overskirt",
					"oxcart",
					"oxygen mask",
					"packet",
					"paddle",
					"paddlewheel",
					"padlock",
					"paintbrush",
					"pajama",
					"palace",
					"panpipe",
					"paper towel",
					"parachute",
					"parallel bars",
					"park bench",
					"parking meter",
					"passenger car",
					"patio",
					"pay-phone",
					"pedestal",
					"pencil box",
					"pencil sharpener",
					"perfume",
					"Petri dish",
					"photocopier",
					"pick",
					"pickelhaube",
					"picket fence",
					"pickup",
					"pier",
					"piggy bank",
					"pill bottle",
					"pillow",
					"ping-pong ball",
					"pinwheel",
					"pirate",
					"pitcher",
					"plane",
					"planetarium",
					"plastic bag",
					"plate rack",
					"plow",
					"plunger",
					"Polaroid camera",
					"pole",
					"police van",
					"poncho",
					"pool table",
					"pop bottle",
					"pot",
					"potter's wheel",
					"power drill",
					"prayer rug",
					"printer",
					"prison",
					"projectile",
					"projector",
					"puck",
					"punching bag",
					"purse",
					"quill",
					"quilt",
					"racer",
					"racket",
					"radiator",
					"radio",
					"radio telescope",
					"rain barrel",
					"recreational vehicle",
					"reel",
					"reflex camera",
					"refrigerator",
					"remote control",
					"restaurant",
					"revolver",
					"rifle",
					"rocking chair",
					"rotisserie",
					"rubber eraser",
					"rugby ball",
					"rule",
					"running shoe",
					"safe",
					"safety pin",
					"saltshaker",
					"sandal",
					"sarong",
					"sax",
					"scabbard",
					"scale",
					"school bus",
					"schooner",
					"scoreboard",
					"screen",
					"screw",
					"screwdriver",
					"seat belt",
					"sewing machine",
					"shield",
					"shoe shop",
					"shoji",
					"shopping basket",
					"shopping cart",
					"shovel",
					"shower cap",
					"shower curtain",
					"ski",
					"ski mask",
					"sleeping bag",
					"slide rule",
					"sliding door",
					"slot",
					"snorkel",
					"snowmobile",
					"snowplow",
					"soap dispenser",
					"soccer ball",
					"sock",
					"solar dish",
					"sombrero",
					"soup bowl",
					"space bar",
					"space heater",
					"space shuttle",
					"spatula",
					"speedboat",
					"spider web",
					"spindle",
					"sports car",
					"spotlight",
					"stage",
					"steam locomotive",
					"steel arch bridge",
					"steel drum",
					"stethoscope",
					"stole",
					"stone wall",
					"stopwatch",
					"stove",
					"strainer",
					"streetcar",
					"stretcher",
					"studio couch",
					"stupa",
					"submarine",
					"suit",
					"sundial",
					"sunglass",
					"sunglasses",
					"sunscreen",
					"suspension bridge",
					"swab",
					"sweatshirt",
					"swimming trunks",
					"swing",
					"switch",
					"syringe",
					"table lamp",
					"tank",
					"tape player",
					"teapot",
					"teddy",
					"television",
					"tennis ball",
					"thatch",
					"theater curtain",
					"thimble",
					"thresher",
					"throne",
					"tile roof",
					"toaster",
					"tobacco shop",
					"toilet seat",
					"torch",
					"totem pole",
					"tow truck",
					"toyshop",
					"tractor",
					"trailer truck",
					"tray",
					"trench coat",
					"tricycle",
					"trimaran",
					"tripod",
					"triumphal arch",
					"trolleybus",
					"trombone",
					"tub",
					"turnstile",
					"typewriter keyboard",
					"umbrella",
					"unicycle",
					"upright",
					"vacuum",
					"vase",
					"vault",
					"velvet",
					"vending machine",
					"vestment",
					"viaduct",
					"violin",
					"volleyball",
					"waffle iron",
					"wall clock",
					"wallet",
					"wardrobe",
					"warplane",
					"washbasin",
					"washer",
					"water bottle",
					"water jug",
					"water tower",
					"whiskey jug",
					"whistle",
					"wig",
					"window screen",
					"window shade",
					"Windsor tie",
					"wine bottle",
					"wing",
					"wok",
					"wooden spoon",
					"wool",
					"worm fence",
					"wreck",
					"yawl",
					"yurt",
					"web site",
					"comic book",
					"crossword puzzle",
					"street sign",
					"traffic light",
					"book jacket",
					"menu",
					"plate",
					"guacamole",
					"consomme",
					"hot pot",
					"trifle",
					"ice cream",
					"ice lolly",
					"French loaf",
					"bagel",
					"pretzel",
					"cheeseburger",
					"hotdog",
					"mashed potato",
					"head cabbage",
					"broccoli",
					"cauliflower",
					"zucchini",
					"spaghetti squash",
					"acorn squash",
					"butternut squash",
					"cucumber",
					"artichoke",
					"bell pepper",
					"cardoon",
					"mushroom",
					"Granny Smith",
					"strawberry",
					"orange",
					"lemon",
					"fig",
					"pineapple",
					"banana",
					"jackfruit",
					"custard apple",
					"pomegranate",
					"hay",
					"carbonara",
					"chocolate sauce",
					"dough",
					"meat loaf",
					"pizza",
					"potpie",
					"burrito",
					"red wine",
					"espresso",
					"cup",
					"eggnog",
					"alp",
					"bubble",
					"cliff",
					"coral reef",
					"geyser",
					"lakeside",
					"promontory",
					"sandbar",
					"seashore",
					"valley",
					"volcano",
					"ballplayer",
					"groom",
					"scuba diver",
					"rapeseed",
					"daisy",
					"yellow lady's slipper",
					"corn",
					"acorn",
					"hip",
					"buckeye",
					"coral fungus",
					"agaric",
					"gyromitra",
					"stinkhorn",
					"earthstar",
					"hen-of-the-woods",
					"bolete",
					"ear",
					"toilet tissue"};
    }

template <typename PointT> void
pcl::people::PersonCluster<PointT>::init (
    const PointCloudPtr& input_cloud,
    const pcl::PointIndices& indices,
    const Eigen::VectorXf& ground_coeffs,
    float sqrt_ground_coeffs,
    bool head_centroid,
    bool vertical)
{

  vertical_ = vertical;
  head_centroid_ = head_centroid;
  person_confidence_ = std::numeric_limits<float>::quiet_NaN();

  min_x_ = 1000.0f;
  min_y_ = 1000.0f;
  min_z_ = 1000.0f;

  max_x_ = -1000.0f;
  max_y_ = -1000.0f;
  max_z_ = -1000.0f;

  sum_x_ = 0.0f;
  sum_y_ = 0.0f;
  sum_z_ = 0.0f;

  n_ = 0;

  points_indices_.indices = indices.indices;

  for (std::vector<int>::const_iterator pit = points_indices_.indices.begin(); pit != points_indices_.indices.end(); pit++)
  {
    PointT* p = &input_cloud->points[*pit];

    min_x_ = std::min(p->x, min_x_);
    max_x_ = std::max(p->x, max_x_);
    sum_x_ += p->x;

    min_y_ = std::min(p->y, min_y_);
    max_y_ = std::max(p->y, max_y_);
    sum_y_ += p->y;

    min_z_ = std::min(p->z, min_z_);
    max_z_ = std::max(p->z, max_z_);
    sum_z_ += p->z;

    n_++;
  }

  c_x_ = sum_x_ / n_;
  c_y_ = sum_y_ / n_;
  c_z_ = sum_z_ / n_;


  Eigen::Vector4f height_point(c_x_, c_y_, c_z_, 1.0f);
  if(!vertical_)
  {
    height_point(1) = min_y_;
    distance_ = std::sqrt(c_x_ * c_x_ + c_z_ * c_z_);
  }
  else
  {
    height_point(0) = max_x_;
    distance_ = std::sqrt(c_y_ * c_y_ + c_z_ * c_z_);
  }

  float height = std::fabs(height_point.dot(ground_coeffs));
  height /= sqrt_ground_coeffs;
  height_ = height;

  if(head_centroid_)
  {
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    int n = 0;

    float head_threshold_value;    // vertical coordinate of the lowest head point
    if (!vertical_)
    {
      head_threshold_value = min_y_ + height_ / 8.0f;    // head is suppose to be 1/8 of the human height
      for (std::vector<int>::const_iterator pit = points_indices_.indices.begin(); pit != points_indices_.indices.end(); pit++)
      {
        PointT* p = &input_cloud->points[*pit];

        if(p->y < head_threshold_value)
        {
          sum_x += p->x;
          sum_y += p->y;
          sum_z += p->z;
          n++;
        }
      }
    }
    else
    {
      head_threshold_value = max_x_ - height_ / 8.0f;    // head is suppose to be 1/8 of the human height
      for (std::vector<int>::const_iterator pit = points_indices_.indices.begin(); pit != points_indices_.indices.end(); pit++)
      {
        PointT* p = &input_cloud->points[*pit];

        if(p->x > head_threshold_value)
        {
          sum_x += p->x;
          sum_y += p->y;
          sum_z += p->z;
          n++;
        }
      }
    }

    c_x_ = sum_x / n;
    c_y_ = sum_y / n;
    c_z_ = sum_z / n;
  }

  if(!vertical_)
  {
    float min_x = c_x_;
    float min_z = c_z_;
    float max_x = c_x_;
    float max_z = c_z_;
    for (std::vector<int>::const_iterator pit = points_indices_.indices.begin(); pit != points_indices_.indices.end(); pit++)
    {
      PointT* p = &input_cloud->points[*pit];

      min_x = std::min(p->x, min_x);
      max_x = std::max(p->x, max_x);
      min_z = std::min(p->z, min_z);
      max_z = std::max(p->z, max_z);
    }

    angle_ = std::atan2(c_z_, c_x_);
    angle_max_ = std::max(std::atan2(min_z, min_x), std::atan2(max_z, min_x));
    angle_min_ = std::min(std::atan2(min_z, max_x), std::atan2(max_z, max_x));

    Eigen::Vector4f c_point(c_x_, c_y_, c_z_, 1.0f);
    float t = c_point.dot(ground_coeffs) / std::pow(sqrt_ground_coeffs, 2);
    float bottom_x = c_x_ - ground_coeffs(0) * t;
    float bottom_y = c_y_ - ground_coeffs(1) * t;
    float bottom_z = c_z_ - ground_coeffs(2) * t;

    tbottom_ = Eigen::Vector3f(bottom_x, bottom_y, bottom_z);
    Eigen::Vector3f v = Eigen::Vector3f(c_x_, c_y_, c_z_) - tbottom_;

    ttop_ = v * height / v.norm() + tbottom_;
    tcenter_ = v * height * 0.5 / v.norm() + tbottom_;
    top_ = Eigen::Vector3f(c_x_, min_y_, c_z_);
    bottom_ = Eigen::Vector3f(c_x_, max_y_, c_z_);
    center_ = Eigen::Vector3f(c_x_, c_y_, c_z_);

    min_ = Eigen::Vector3f(min_x_, min_y_, min_z_);

    max_ = Eigen::Vector3f(max_x_, max_y_, max_z_);
  }
  else
  {
    float min_y = c_y_;
    float min_z = c_z_;
    float max_y = c_y_;
    float max_z = c_z_;
    for (std::vector<int>::const_iterator pit = points_indices_.indices.begin(); pit != points_indices_.indices.end(); pit++)
    {
      PointT* p = &input_cloud->points[*pit];

      min_y = std::min(p->y, min_y);
      max_y = std::max(p->y, max_y);
      min_z = std::min(p->z, min_z);
      max_z = std::max(p->z, max_z);
    }

    angle_ = std::atan2(c_z_, c_y_);
    angle_max_ = std::max(std::atan2(min_z_, min_y_), std::atan2(max_z_, min_y_));
    angle_min_ = std::min(std::atan2(min_z_, max_y_), std::atan2(max_z_, max_y_));

    Eigen::Vector4f c_point(c_x_, c_y_, c_z_, 1.0f);
    float t = c_point.dot(ground_coeffs) / std::pow(sqrt_ground_coeffs, 2);
    float bottom_x = c_x_ - ground_coeffs(0) * t;
    float bottom_y = c_y_ - ground_coeffs(1) * t;
    float bottom_z = c_z_ - ground_coeffs(2) * t;

    tbottom_ = Eigen::Vector3f(bottom_x, bottom_y, bottom_z);
    Eigen::Vector3f v = Eigen::Vector3f(c_x_, c_y_, c_z_) - tbottom_;

    ttop_ = v * height / v.norm() + tbottom_;
    tcenter_ = v * height * 0.5 / v.norm() + tbottom_;
    top_ = Eigen::Vector3f(max_x_, c_y_, c_z_);
    bottom_ = Eigen::Vector3f(min_x_, c_y_, c_z_);
    center_ = Eigen::Vector3f(c_x_, c_y_, c_z_);

    min_ = Eigen::Vector3f(min_x_, min_y_, min_z_);

    max_ = Eigen::Vector3f(max_x_, max_y_, max_z_);
  }
}

template <typename PointT> pcl::PointIndices&
pcl::people::PersonCluster<PointT>::getIndices ()
{
  return (points_indices_);
}

template <typename PointT> float
pcl::people::PersonCluster<PointT>::getHeight ()
{
  return (height_);
}

template <typename PointT> float
pcl::people::PersonCluster<PointT>::updateHeight (const Eigen::VectorXf& ground_coeffs)
{
  float sqrt_ground_coeffs = (ground_coeffs - Eigen::Vector4f(0.0f, 0.0f, 0.0f, ground_coeffs(3))).norm();
  return (updateHeight(ground_coeffs, sqrt_ground_coeffs));
}

template <typename PointT> float
pcl::people::PersonCluster<PointT>::updateHeight (const Eigen::VectorXf& ground_coeffs, float sqrt_ground_coeffs)
{
  Eigen::Vector4f height_point;
  if (!vertical_)
    height_point << c_x_, min_y_, c_z_, 1.0f;
  else
    height_point << max_x_, c_y_, c_z_, 1.0f;

  float height = std::fabs(height_point.dot(ground_coeffs));
  height /= sqrt_ground_coeffs;
  height_ = height;
  return (height_);
}

template <typename PointT> float
pcl::people::PersonCluster<PointT>::getDistance ()
{
  return (distance_);
}

template <typename PointT> Eigen::Vector3f&
pcl::people::PersonCluster<PointT>::getTTop ()
{
  return (ttop_);
}

template <typename PointT> Eigen::Vector3f&
pcl::people::PersonCluster<PointT>::getTBottom ()
{
  return (tbottom_);
}

template <typename PointT> Eigen::Vector3f&
pcl::people::PersonCluster<PointT>::getTCenter ()
{
  return (tcenter_);
}

template <typename PointT> Eigen::Vector3f&
pcl::people::PersonCluster<PointT>::getTop ()
{
  return (top_);
}

template <typename PointT> Eigen::Vector3f&
pcl::people::PersonCluster<PointT>::getBottom ()
{
  return (bottom_);
}

template <typename PointT> Eigen::Vector3f&
pcl::people::PersonCluster<PointT>::getCenter ()
{
  return (center_);
}

template <typename PointT> Eigen::Vector3f&
pcl::people::PersonCluster<PointT>::getMin ()
{
  return (min_);
}

template <typename PointT> Eigen::Vector3f&
pcl::people::PersonCluster<PointT>::getMax ()
{
  return (max_);
}

template <typename PointT> float
pcl::people::PersonCluster<PointT>::getAngle ()
{
  return (angle_);
}

template <typename PointT>
float pcl::people::PersonCluster<PointT>::getAngleMax ()
{
  return (angle_max_);
}

template <typename PointT>
float pcl::people::PersonCluster<PointT>::getAngleMin ()
{
  return (angle_min_);
}

template <typename PointT>
int pcl::people::PersonCluster<PointT>::getNumberPoints ()
{
  return (n_);
}

template <typename PointT>
float pcl::people::PersonCluster<PointT>::getPersonConfidence ()
{
  return (person_confidence_);
}

template <typename PointT>
void pcl::people::PersonCluster<PointT>::setPersonConfidence (float confidence)
{
  person_confidence_ = confidence;
}

template <typename PointT>
void pcl::people::PersonCluster<PointT>::setObjectConfidence (float result[])
{
  object_confidence_[1] = result[1];
  object_confidence_[0] = result[0];
}

template <typename PointT>
void pcl::people::PersonCluster<PointT>::setHeight (float height)
{
  height_ = height;
}

template <typename PointT>
void pcl::people::PersonCluster<PointT>::drawTBoundingBox (pcl::visualization::PCLVisualizer& viewer, int person_number)
{
  // draw theoretical person bounding box in the PCL viewer:
  pcl::ModelCoefficients coeffs;
  // translation
  coeffs.values.push_back (tcenter_[0]);
  coeffs.values.push_back (tcenter_[1]);
  coeffs.values.push_back (tcenter_[2]);
  // rotation
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  // size
  if (vertical_)
  {
    coeffs.values.push_back (height_);
    coeffs.values.push_back (0.5);
    coeffs.values.push_back (0.5);
  }
  else
  {
    coeffs.values.push_back (0.5);
    coeffs.values.push_back (height_);
    coeffs.values.push_back (0.5);
  }

  std::stringstream bbox_name;
  bbox_name << "bbox_person_" << person_number;
  viewer.removeShape (bbox_name.str());
  viewer.addCube (coeffs, bbox_name.str());
  viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, bbox_name.str());
  viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, bbox_name.str());

  //      std::stringstream confid;
  //      confid << person_confidence_;
  //      PointT position;
  //      position.x = tcenter_[0]- 0.2;
  //      position.y = ttop_[1];
  //      position.z = tcenter_[2];
  //      viewer.addText3D(confid.str().substr(0, 4), position, 0.1);
}


template <typename PointT>
void pcl::people::PersonCluster<PointT>::drawTBoundingBoxObject (pcl::visualization::PCLVisualizer& viewer, int object_number)
{
  // draw theoretical person bounding box in the PCL viewer:
  pcl::ModelCoefficients coeffs;
  // translation
  coeffs.values.push_back (tcenter_[0]);
  coeffs.values.push_back (tcenter_[1]);
  coeffs.values.push_back (tcenter_[2]);
  // rotation
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  // size
  if (vertical_)
  {
    coeffs.values.push_back (height_);
    coeffs.values.push_back (0.5);
    coeffs.values.push_back (0.5);
  }
  else
  {
    coeffs.values.push_back (0.5);
    coeffs.values.push_back (height_);
    coeffs.values.push_back (0.5);
  }

  std::stringstream bbox_name;
  bbox_name << "bbox_person_" << person_number;
  viewer.removeShape (bbox_name.str());
  viewer.addCube (coeffs, bbox_name.str());
  viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, bbox_name.str());
  viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, bbox_name.str());

       std::stringstream confid;
       confid << person_confidence_[1] << classList_[person_confidence_[0]];
       PointT position;
       position.x = tcenter_[0]- 0.2;
       position.y = ttop_[1];
       position.z = tcenter_[2];
       viewer.addText3D(, position, 0.1);

      //  std::stringstream classObject;
      //  classObject << classList_[person_confidence_[0]];
      //  PointT position;
      //  position.x = tcenter_[0]- 0.2;
      //  position.y = ttop_[1];
      //  position.z = tcenter_[2];
      //  viewer.addText3D(, position, 0.1);
}


template <typename PointT>
pcl::people::PersonCluster<PointT>::~PersonCluster ()
{
  // Auto-generated destructor stub
}
#endif /* PCL_PEOPLE_PERSON_CLUSTER_HPP_ */
