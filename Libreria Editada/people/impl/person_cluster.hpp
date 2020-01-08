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
      object_confidence_ = new float[2];
      classList_ = { 'tench, Tinca tinca',
  'goldfish, Carassius auratus',
  'great white shark, white shark, man-eater, man-eating shark, Carcharodon carcharias',
  'tiger shark, Galeocerdo cuvieri',
  'hammerhead, hammerhead shark',
  'electric ray, crampfish, numbfish, torpedo',
  'stingray',
  'cock',
  'hen',
  'ostrich, Struthio camelus',
  'brambling, Fringilla montifringilla',
 'goldfinch, Carduelis carduelis',
 'house finch, linnet, Carpodacus mexicanus',
 'junco, snowbird',
 'indigo bunting, indigo finch, indigo bird, Passerina cyanea',
 'robin, American robin, Turdus migratorius',
 'bulbul',
 'jay',
 'magpie',
 'chickadee',
 'water ouzel, dipper',
 'kite',
 'bald eagle, American eagle, Haliaeetus leucocephalus',
 'vulture',
 'great grey owl, great gray owl, Strix nebulosa',
 'European fire salamander, Salamandra salamandra',
 'common newt, Triturus vulgaris',
 'eft',
 'spotted salamander, Ambystoma maculatum',
 'axolotl, mud puppy, Ambystoma mexicanum',
 'bullfrog, Rana catesbeiana',
 'tree frog, tree-frog',
 'tailed frog, bell toad, ribbed toad, tailed toad, Ascaphus trui',
 'loggerhead, loggerhead turtle, Caretta caretta',
 'leatherback turtle, leatherback, leathery turtle, Dermochelys coriacea',
 'mud turtle',
 'terrapin',
 'box turtle, box tortoise',
 'banded gecko',
 'common iguana, iguana, Iguana iguana',
 'American chameleon, anole, Anolis carolinensis',
 'whiptail, whiptail lizard',
 'agama',
 'frilled lizard, Chlamydosaurus kingi',
 'alligator lizard',
 'Gila monster, Heloderma suspectum',
 'green lizard, Lacerta viridis',
 'African chameleon, Chamaeleo chamaeleon',
 'Komodo dragon, Komodo lizard, dragon lizard, giant lizard, Varanus komodoensis',
 'African crocodile, Nile crocodile, Crocodylus niloticus',
 'American alligator, Alligator mississipiensis',
 'triceratops',
 'thunder snake, worm snake, Carphophis amoenus',
 'ringneck snake, ring-necked snake, ring snake',
 'hognose snake, puff adder, sand viper',
 'green snake, grass snake',
 'king snake, kingsnake',
 'garter snake, grass snake',
 'water snake',
 'vine snake',
 'night snake, Hypsiglena torquata',
 'boa constrictor, Constrictor constrictor',
 'rock python, rock snake, Python sebae',
 'Indian cobra, Naja naja',
 'green mamba',
 'sea snake',
 'horned viper, cerastes, sand viper, horned asp, Cerastes cornutus',
 'diamondback, diamondback rattlesnake, Crotalus adamanteus',
 'sidewinder, horned rattlesnake, Crotalus cerastes',
 'trilobite',
 'harvestman, daddy longlegs, Phalangium opilio',
 'scorpion',
 'black and gold garden spider, Argiope aurantia',
 'barn spider, Araneus cavaticus',
 'garden spider, Aranea diademata',
 'black widow, Latrodectus mactans',
 'tarantula',
 'wolf spider, hunting spider',
 'tick',
 'centipede',
 'black grouse',
 'ptarmigan',
 'ruffed grouse, partridge, Bonasa umbellus',
 'prairie chicken, prairie grouse, prairie fowl',
 'peacock',
 'quail',
 'partridge',
 'African grey, African gray, Psittacus erithacus',
 'macaw',
 'sulphur-crested cockatoo, Kakatoe galerita, Cacatua galerita',
 'lorikeet',
 'coucal',
 'bee eater',
 'hornbill',
 'hummingbird',
 'jacamar',
 'toucan',
 'drake',
 'red-breasted merganser, Mergus serrator',
 'goose',
  'black swan, Cygnus atratus',
  'tusker',
  'echidna, spiny anteater, anteater',
  'platypus, duckbill, duckbilled platypus, duck-billed platypus, Ornithorhynchus anatinus',
  'wallaby, brush kangaroo',
  'koala, koala bear, kangaroo bear, native bear, Phascolarctos cinereus',
  'wombat',
  'jellyfish',
  'sea anemone, anemone',
  'brain coral',
  'flatworm, platyhelminth',
  'nematode, nematode worm, roundworm',
  'conch',
  'snail',
  'slug',
  'sea slug, nudibranch',
  'chiton, coat-of-mail shell, sea cradle, polyplacophore',
  'chambered nautilus, pearly nautilus, nautilus',
  'Dungeness crab, Cancer magister',
  'rock crab, Cancer irroratus',
  'fiddler crab',
  'king crab, Alaska crab, Alaskan king crab, Alaska king crab, Paralithodes camtschatica',
  'American lobster, Northern lobster, Maine lobster, Homarus americanus',
  'spiny lobster, langouste, rock lobster, crawfish, crayfish, sea crawfish',
  'crayfish, crawfish, crawdad, crawdaddy',
  'hermit crab',
  'isopod',
  'white stork, Ciconia ciconia',
  'black stork, Ciconia nigra',
  'spoonbill',
  'flamingo',
  'little blue heron, Egretta caerulea',
  'American egret, great white heron, Egretta albus',
  'bittern',
  'crane',
  'limpkin, Aramus pictus',
  'European gallinule, Porphyrio porphyrio',
  'American coot, marsh hen, mud hen, water hen, Fulica americana',
  'bustard',
  'ruddy turnstone, Arenaria interpres',
  'red-backed sandpiper, dunlin, Erolia alpina',
  'redshank, Tringa totanus',
  'dowitcher',
  'oystercatcher, oyster catcher',
  'pelican',
  'king penguin, Aptenodytes patagonica',
  'albatross, mollymawk',
  'grey whale, gray whale, devilfish, Eschrichtius gibbosus, Eschrichtius robustus',
  'killer whale, killer, orca, grampus, sea wolf, Orcinus orca',
  'dugong, Dugong dugon',
  'sea lion',
  'Chihuahua',
  'Japanese spaniel',
  'Maltese dog, Maltese terrier, Maltese',
  'Pekinese, Pekingese, Peke',
  'Shih-Tzu',
  'Blenheim spaniel',
  'papillon',
  'toy terrier',
  'Rhodesian ridgeback',
  'Afghan hound, Afghan',
  'basset, basset hound',
  'beagle',
  'bloodhound, sleuthhound',
  'bluetick',
  'black-and-tan coonhound',
  'Walker hound, Walker foxhound',
  'English foxhound',
  'redbone',
  'borzoi, Russian wolfhound',
  'Irish wolfhound',
  'Italian greyhound',
  'whippet',
  'Ibizan hound, Ibizan Podenco',
  'Norwegian elkhound, elkhound',
  'otterhound, otter hound',
  'Saluki, gazelle hound',
  'Scottish deerhound, deerhound',
  'Weimaraner',
  'Staffordshire bullterrier, Staffordshire bull terrier',
  'American Staffordshire terrier, Staffordshire terrier, American pit bull terrier, pit bull terrier',
  'Bedlington terrier',
  'Border terrier',
  'Kerry blue terrier',
  'Irish terrier',
  'Norfolk terrier',
  'Norwich terrier',
  'Yorkshire terrier',
  'wire-haired fox terrier',
  'Lakeland terrier',
  'Sealyham terrier, Sealyham',
  'Airedale, Airedale terrier',
  'cairn, cairn terrier',
  'Australian terrier',
  'Dandie Dinmont, Dandie Dinmont terrier',
  'Boston bull, Boston terrier',
  'miniature schnauzer',
  'giant schnauzer',
  'standard schnauzer',
  'Scotch terrier, Scottish terrier, Scottie',
  'Tibetan terrier, chrysanthemum dog',
  'silky terrier, Sydney silky',
  'soft-coated wheaten terrier',
  'West Highland white terrier',
  'Lhasa, Lhasa apso',
  'flat-coated retriever',
  'curly-coated retriever',
  'golden retriever',
  'Labrador retriever',
  'Chesapeake Bay retriever',
  'German short-haired pointer',
  'vizsla, Hungarian pointer',
  'English setter',
  'Irish setter, red setter',
  'Gordon setter',
  'Brittany spaniel',
  'clumber, clumber spaniel',
  'English springer, English springer spaniel',
  'Welsh springer spaniel',
  'cocker spaniel, English cocker spaniel, cocker',
  'Sussex spaniel',
  'Irish water spaniel',
  'kuvasz',
  'schipperke',
  'groenendael',
  'malinois',
  'briard',
  'kelpie',
  'komondor',
  'Old English sheepdog, bobtail',
  'Shetland sheepdog, Shetland sheep dog, Shetland',
  'collie',
  'Border collie',
  'Bouvier des Flandres, Bouviers des Flandres',
  'Rottweiler',
  'German shepherd, German shepherd dog, German police dog, alsatian',
  'Doberman, Doberman pinscher',
  'miniature pinscher',
  'Greater Swiss Mountain dog',
  'Bernese mountain dog',
  'Appenzeller',
  'EntleBucher',
  'boxer',
  'bull mastiff',
  'Tibetan mastiff',
  'French bulldog',
  'Great Dane',
  'Saint Bernard, St Bernard',
  'Eskimo dog, husky',
  'malamute, malemute, Alaskan malamute',
  'Siberian husky',
  'dalmatian, coach dog, carriage dog',
  'affenpinscher, monkey pinscher, monkey dog',
  'basenji',
  'pug, pug-dog',
  'Leonberg',
  'Newfoundland, Newfoundland dog',
  'Great Pyrenees',
  'Samoyed, Samoyede',
  'Pomeranian',
  'chow, chow chow',
  'keeshond',
  'Brabancon griffon',
  'Pembroke, Pembroke Welsh corgi',
  'Cardigan, Cardigan Welsh corgi',
  'toy poodle',
  'miniature poodle',
  'standard poodle',
  'Mexican hairless',
  'timber wolf, grey wolf, gray wolf, Canis lupus',
  'white wolf, Arctic wolf, Canis lupus tundrarum',
  'red wolf, maned wolf, Canis rufus, Canis niger',
  'coyote, prairie wolf, brush wolf, Canis latrans',
  'dingo, warrigal, warragal, Canis dingo',
  'dhole, Cuon alpinus',
  'African hunting dog, hyena dog, Cape hunting dog, Lycaon pictus',
  'hyena, hyaena',
  'red fox, Vulpes vulpes',
  'kit fox, Vulpes macrotis',
  'Arctic fox, white fox, Alopex lagopus',
  'grey fox, gray fox, Urocyon cinereoargenteus',
  'tabby, tabby cat',
  'tiger cat',
  'Persian cat',
  'Siamese cat, Siamese',
  'Egyptian cat',
  'cougar, puma, catamount, mountain lion, painter, panther, Felis concolor',
  'lynx, catamount',
  'leopard, Panthera pardus',
  'snow leopard, ounce, Panthera uncia',
  'jaguar, panther, Panthera onca, Felis onca',
  'lion, king of beasts, Panthera leo',
  'tiger, Panthera tigris',
  'cheetah, chetah, Acinonyx jubatus',
  'brown bear, bruin, Ursus arctos',
  'American black bear, black bear, Ursus americanus, Euarctos americanus',
  'ice bear, polar bear, Ursus Maritimus, Thalarctos maritimus',
  'sloth bear, Melursus ursinus, Ursus ursinus',
  'mongoose',
  'meerkat, mierkat',
  'tiger beetle',
  'ladybug, ladybeetle, lady beetle, ladybird, ladybird beetle',
  'ground beetle, carabid beetle',
  'long-horned beetle, longicorn, longicorn beetle',
  'leaf beetle, chrysomelid',
  'dung beetle',
  'rhinoceros beetle',
  'weevil',
  'fly',
  'bee',
  'ant, emmet, pismire',
  'grasshopper, hopper',
  'cricket',
  'walking stick, walkingstick, stick insect',
  'cockroach, roach',
  'mantis, mantid',
  'cicada, cicala',
  'leafhopper',
  'lacewing, lacewing fly',
  "dragonfly, darning needle, devil's darning needle, sewing needle, snake feeder, snake doctor, mosquito hawk, skeeter hawk",
  'damselfly',
  'admiral',
  'ringlet, ringlet butterfly',
  'monarch, monarch butterfly, milkweed butterfly, Danaus plexippus',
  'cabbage butterfly',
  'sulphur butterfly, sulfur butterfly',
  'lycaenid, lycaenid butterfly',
  'starfish, sea star',
  'sea urchin',
  'sea cucumber, holothurian',
  'wood rabbit, cottontail, cottontail rabbit',
  'hare',
  'Angora, Angora rabbit',
  'hamster',
  'porcupine, hedgehog',
  'fox squirrel, eastern fox squirrel, Sciurus niger',
  'marmot',
  'beaver',
  'guinea pig, Cavia cobaya',
  'sorrel',
  'zebra',
  'hog, pig, grunter, squealer, Sus scrofa',
  'wild boar, boar, Sus scrofa',
  'warthog',
  'hippopotamus, hippo, river horse, Hippopotamus amphibius',
  'ox',
  'water buffalo, water ox, Asiatic buffalo, Bubalus bubalis',
  'bison',
  'ram, tup',
  'bighorn, bighorn sheep, cimarron, Rocky Mountain bighorn, Rocky Mountain sheep, Ovis canadensis',
  'ibex, Capra ibex',
  'hartebeest',
  'impala, Aepyceros melampus',
  'gazelle',
  'Arabian camel, dromedary, Camelus dromedarius',
  'llama',
  'weasel',
  'mink',
  'polecat, fitch, foulmart, foumart, Mustela putorius',
  'black-footed ferret, ferret, Mustela nigripes',
  'otter',
  'skunk, polecat, wood pussy',
  'badger',
  'armadillo',
  'three-toed sloth, ai, Bradypus tridactylus',
  'orangutan, orang, orangutang, Pongo pygmaeus',
  'gorilla, Gorilla gorilla',
  'chimpanzee, chimp, Pan troglodytes',
  'gibbon, Hylobates lar',
  'siamang, Hylobates syndactylus, Symphalangus syndactylus',
  'guenon, guenon monkey',
  'patas, hussar monkey, Erythrocebus patas',
  'baboon',
  'macaque',
  'langur',
  'colobus, colobus monkey',
  'proboscis monkey, Nasalis larvatus',
  'marmoset',
  'capuchin, ringtail, Cebus capucinus',
  'howler monkey, howler',
  'titi, titi monkey',
  'spider monkey, Ateles geoffroyi',
  'squirrel monkey, Saimiri sciureus',
  'Madagascar cat, ring-tailed lemur, Lemur catta',
  'indri, indris, Indri indri, Indri brevicaudatus',
  'Indian elephant, Elephas maximus',
  'African elephant, Loxodonta africana',
  'lesser panda, red panda, panda, bear cat, cat bear, Ailurus fulgens',
  'giant panda, panda, panda bear, coon bear, Ailuropoda melanoleuca',
  'barracouta, snoek',
  'eel',
  'coho, cohoe, coho salmon, blue jack, silver salmon, Oncorhynchus kisutch',
  'rock beauty, Holocanthus tricolor',
  'anemone fish',
  'sturgeon',
  'gar, garfish, garpike, billfish, Lepisosteus osseus',
  'lionfish',
  'puffer, pufferfish, blowfish, globefish',
  'abacus',
  'abaya',
  "academic gown, academic robe, judge's robe",
  'accordion, piano accordion, squeeze box',
  'acoustic guitar',
  'aircraft carrier, carrier, flattop, attack aircraft carrier',
  'airliner',
  'airship, dirigible',
  'altar',
  'ambulance',
  'amphibian, amphibious vehicle',
  'analog clock',
  'apiary, bee house',
  'apron',
  'ashcan, trash can, garbage can, wastebin, ash bin, ash-bin, ashbin, dustbin, trash barrel, trash bin',
  'assault rifle, assault gun',
  'backpack, back pack, knapsack, packsack, rucksack, haversack',
  'bakery, bakeshop, bakehouse',
  'balance beam, beam',
  'balloon',
  'ballpoint, ballpoint pen, ballpen, Biro',
  'Band Aid',
  'banjo',
  'bannister, banister, balustrade, balusters, handrail',
  'barbell',
  'barber chair',
  'barbershop',
  'barn',
  'barometer',
  'barrel, cask',
  'barrow, garden cart, lawn cart, wheelbarrow',
  'baseball',
  'basketball',
  'bassinet',
  'bassoon',
  'bathing cap, swimming cap',
  'bath towel',
  'bathtub, bathing tub, bath, tub',
  'beach wagon, station wagon, wagon, estate car, beach waggon, station waggon, waggon',
  'beacon, lighthouse, beacon light, pharos',
  'beaker',
  'bearskin, busby, shako',
  'beer bottle',
  'beer glass',
  'bell cote, bell cot',
  'bib',
  'bicycle-built-for-two, tandem bicycle, tandem',
  'bikini, two-piece',
  'binder, ring-binder',
  'binoculars, field glasses, opera glasses',
  'birdhouse',
  'boathouse',
  'bobsled, bobsleigh, bob',
  'bolo tie, bolo, bola tie, bola',
  'bonnet, poke bonnet',
  'bookcase',
  'bookshop, bookstore, bookstall',
  'bottlecap',
  'bow',
  'bow tie, bow-tie, bowtie',
  'brass, memorial tablet, plaque',
  'brassiere, bra, bandeau',
  'breakwater, groin, groyne, mole, bulwark, seawall, jetty',
  'breastplate, aegis, egis',
  'broom',
  'bucket, pail',
  'buckle',
  'bulletproof vest',
  'bullet train, bullet',
  'butcher shop, meat market',
  'cab, hack, taxi, taxicab',
  'caldron, cauldron',
  'candle, taper, wax light',
  'cannon',
  'canoe',
  'can opener, tin opener',
  'cardigan',
  'car mirror',
  'carousel, carrousel, merry-go-round, roundabout, whirligig',
  "carpenter's kit, tool kit",
  'carton',
  'car wheel',
  'cash machine, cash dispenser, automated teller machine, automatic teller machine, automated teller, automatic teller, ATM',
  'cassette',
  'cassette player',
  'castle',
  'catamaran',
  'CD player',
  'cello, violoncello',
  'cellular telephone, cellular phone, cellphone, cell, mobile phone',
  'chain',
  'chainlink fence',
  'chain mail, ring mail, mail, chain armor, chain armour, ring armor, ring armour',
  'chain saw, chainsaw',
  'chest',
  'chiffonier, commode',
  'chime, bell, gong',
  'china cabinet, china closet',
  'Christmas stocking',
  'church, church building',
  'cinema, movie theater, movie theatre, movie house, picture palace',
  'cleaver, meat cleaver, chopper',
  'cliff dwelling',
  'cloak',
  'clog, geta, patten, sabot',
  'cocktail shaker',
  'coffee mug',
  'coffeepot',
  'coil, spiral, volute, whorl, helix',
  'combination lock',
  'computer keyboard, keypad',
  'confectionery, confectionary, candy store',
  'container ship, containership, container vessel',
  'convertible',
  'corkscrew, bottle screw',
  'cornet, horn, trumpet, trump',
  'cowboy boot',
  'cowboy hat, ten-gallon hat',
  'cradle',
  'crane',
  'crash helmet',
  'crate',
  'crib, cot',
  'Crock Pot',
  'croquet ball',
  'crutch',
  'cuirass',
  'dam, dike, dyke',
  'desk',
  'desktop computer',
  'dial telephone, dial phone',
  'diaper, nappy, napkin',
  'digital clock',
  'digital watch',
  'dining table, board',
  'dishrag, dishcloth',
  'dishwasher, dish washer, dishwashing machine',
  'disk brake, disc brake',
  'dock, dockage, docking facility',
  'dogsled, dog sled, dog sleigh',
  'dome',
  'doormat, welcome mat',
  'drilling platform, offshore rig',
  'drum, membranophone, tympan',
  'drumstick',
  'dumbbell',
  'Dutch oven',
  'electric fan, blower',
  'electric guitar',
  'electric locomotive',
  'entertainment center',
  'envelope',
  'espresso maker',
  'face powder',
  'feather boa, boa',
  'file, file cabinet, filing cabinet',
  'fireboat',
  'fire engine, fire truck',
  'fire screen, fireguard',
  'flagpole, flagstaff',
  'flute, transverse flute',
  'folding chair',
  'football helmet',
  'forklift',
  'fountain',
  'fountain pen',
  'four-poster',
  'freight car',
  'French horn, horn',
  'frying pan, frypan, skillet',
  'fur coat',
  'garbage truck, dustcart',
  'gasmask, respirator, gas helmet',
  'gas pump, gasoline pump, petrol pump, island dispenser',
  'goblet',
  'go-kart',
  'golf ball',
  'golfcart, golf cart',
  'gondola',
  'gong, tam-tam',
  'gown',
  'grand piano, grand',
  'greenhouse, nursery, glasshouse',
  'grille, radiator grille',
  'grocery store, grocery, food market, market',
  'guillotine',
  'hair slide',
  'hair spray',
  'half track',
  'hammer',
  'hamper',
  'hand blower, blow dryer, blow drier, hair dryer, hair drier',
  'hand-held computer, hand-held microcomputer',
  'handkerchief, hankie, hanky, hankey',
  'hard disc, hard disk, fixed disk',
  'harmonica, mouth organ, harp, mouth harp',
  'harp',
  'harvester, reaper',
  'hatchet',
  'holster',
  'home theater, home theatre',
  'honeycomb',
  'hook, claw',
  'hoopskirt, crinoline',
  'horizontal bar, high bar',
  'horse cart, horse-cart',
  'hourglass',
  'iPod',
  'iron, smoothing iron',
  "jack-o'-lantern",
  'jean, blue jean, denim',
  'jeep, landrover',
  'jersey, T-shirt, tee shirt',
  'jigsaw puzzle',
  'jinrikisha, ricksha, rickshaw',
  'joystick',
  'kimono',
  'knee pad',
  'knot',
  'lab coat, laboratory coat',
  'ladle',
  'lampshade, lamp shade',
  'laptop, laptop computer',
  'lawn mower, mower',
  'lens cap, lens cover',
  'letter opener, paper knife, paperknife',
  'library',
  'lifeboat',
  'lighter, light, igniter, ignitor',
  'limousine, limo',
  'liner, ocean liner',
  'lipstick, lip rouge',
  'Loafer',
  'lotion',
  'loudspeaker, speaker, speaker unit, loudspeaker system, speaker system',
  "loupe, jeweler's loupe",
  'lumbermill, sawmill',
  'magnetic compass',
  'mailbag, postbag',
  'mailbox, letter box',
  'maillot',
  'maillot, tank suit',
  'manhole cover',
  'maraca',
  'marimba, xylophone',
  'mask',
  'matchstick',
  'maypole',
  'maze, labyrinth',
  'measuring cup',
  'medicine chest, medicine cabinet',
  'megalith, megalithic structure',
  'microphone, mike',
  'microwave, microwave oven',
  'military uniform',
  'milk can',
  'minibus',
  'miniskirt, mini',
  'minivan',
  'missile',
  'mitten',
  'mixing bowl',
  'mobile home, manufactured home',
  'Model T',
  'modem',
  'monastery',
  'monitor',
  'moped',
  'mortar',
  'mortarboard',
  'mosque',
  'mosquito net',
  'motor scooter, scooter',
  'mountain bike, all-terrain bike, off-roader',
  'mountain tent',
  'mouse, computer mouse',
  'mousetrap',
  'moving van',
  'muzzle',
  'nail',
  'neck brace',
  'necklace',
  'nipple',
  'notebook, notebook computer',
  'obelisk',
  'oboe, hautboy, hautbois',
  'ocarina, sweet potato',
  'odometer, hodometer, mileometer, milometer',
  'oil filter',
  'organ, pipe organ',
  'oscilloscope, scope, cathode-ray oscilloscope, CRO',
  'overskirt',
  'oxcart',
  'oxygen mask',
  'packet',
  'paddle, boat paddle',
  'paddlewheel, paddle wheel',
  'padlock',
  'paintbrush',
  "pajama, pyjama, pj's, jammies",
  'palace',
  'panpipe, pandean pipe, syrinx',
  'paper towel',
  'parachute, chute',
  'parallel bars, bars',
  'park bench',
  'parking meter',
  'passenger car, coach, carriage',
  'patio, terrace',
  'pay-phone, pay-station',
  'pedestal, plinth, footstall',
  'pencil box, pencil case',
  'pencil sharpener',
  'perfume, essence',
  'Petri dish',
  'photocopier',
  'pick, plectrum, plectron',
  'pickelhaube',
  'picket fence, paling',
  'pickup, pickup truck',
  'pier',
  'piggy bank, penny bank',
  'pill bottle',
  'pillow',
  'ping-pong ball',
  'pinwheel',
  'pirate, pirate ship',
  'pitcher, ewer',
  "plane, carpenter's plane, woodworking plane",
  'planetarium',
  'plastic bag',
  'plate rack',
  'plow, plough',
  "plunger, plumber's helper",
  'Polaroid camera, Polaroid Land camera',
  'pole',
  'police van, police wagon, paddy wagon, patrol wagon, wagon, black Maria',
  'poncho',
  'pool table, billiard table, snooker table',
  'pop bottle, soda bottle',
  'pot, flowerpot',
  "potter's wheel",
  'power drill',
  'prayer rug, prayer mat',
  'printer',
  'prison, prison house',
  'projectile, missile',
  'projector',
  'puck, hockey puck',
  'punching bag, punch bag, punching ball, punchball',
  'purse',
  'quill, quill pen',
  'quilt, comforter, comfort, puff',
  'racer, race car, racing car',
  'racket, racquet',
  'radiator',
  'radio, wireless',
  'radio telescope, radio reflector',
  'rain barrel',
  'recreational vehicle, RV, R.V.',
  'reel',
  'reflex camera',
  'refrigerator, icebox',
  'remote control, remote',
  'restaurant, eating house, eating place, eatery',
  'revolver, six-gun, six-shooter',
  'rifle',
  'rocking chair, rocker',
  'rotisserie',
  'rubber eraser, rubber, pencil eraser',
  'rugby ball',
  'rule, ruler',
  'running shoe',
  'safe',
  'safety pin',
  'saltshaker, salt shaker',
  'sandal',
  'sarong',
  'sax, saxophone',
  'scabbard',
  'scale, weighing machine',
  'school bus',
  'schooner',
  'scoreboard',
  'screen, CRT screen',
  'screw',
  'screwdriver',
  'seat belt, seatbelt',
  'sewing machine',
  'shield, buckler',
  'shoe shop, shoe-shop, shoe store',
  'shoji',
  'shopping basket',
  'shopping cart',
  'shovel',
  'shower cap',
  'shower curtain',
  'ski',
  'ski mask',
  'sleeping bag',
  'slide rule, slipstick',
  'sliding door',
  'slot, one-armed bandit',
  'snorkel',
  'snowmobile',
  'snowplow, snowplough',
  'soap dispenser',
  'soccer ball',
  'sock',
  'solar dish, solar collector, solar furnace',
  'sombrero',
  'soup bowl',
  'space bar',
  'space heater',
  'space shuttle',
  'spatula',
  'speedboat',
  "spider web, spider's web",
  'spindle',
  'sports car, sport car',
  'spotlight, spot',
  'stage',
  'steam locomotive',
  'steel arch bridge',
  'steel drum',
  'stethoscope',
  'stole',
  'stone wall',
  'stopwatch, stop watch',
  'stove',
  'strainer',
  'streetcar, tram, tramcar, trolley, trolley car',
  'stretcher',
  'studio couch, day bed',
  'stupa, tope',
  'submarine, pigboat, sub, U-boat',
  'suit, suit of clothes',
  'sundial',
  'sunglass',
  'sunglasses, dark glasses, shades',
  'sunscreen, sunblock, sun blocker',
  'suspension bridge',
  'swab, swob, mop',
  'sweatshirt',
  'swimming trunks, bathing trunks',
  'swing',
  'switch, electric switch, electrical switch',
  'syringe',
  'table lamp',
  'tank, army tank, armored combat vehicle, armoured combat vehicle',
  'tape player',
  'teapot',
  'teddy, teddy bear',
  'television, television system',
  'tennis ball',
  'thatch, thatched roof',
  'theater curtain, theatre curtain',
  'thimble',
  'thresher, thrasher, threshing machine',
  'throne',
  'tile roof',
  'toaster',
  'tobacco shop, tobacconist shop, tobacconist',
  'toilet seat',
  'torch',
  'totem pole',
  'tow truck, tow car, wrecker',
  'toyshop',
  'tractor',
  'trailer truck, tractor trailer, trucking rig, rig, articulated lorry, semi',
  'tray',
  'trench coat',
  'tricycle, trike, velocipede',
  'trimaran',
  'tripod',
  'triumphal arch',
  'trolleybus, trolley coach, trackless trolley',
  'trombone',
  'tub, vat',
  'turnstile',
  'typewriter keyboard',
  'umbrella',
  'unicycle, monocycle',
  'upright, upright piano',
  'vacuum, vacuum cleaner',
  'vase',
  'vault',
  'velvet',
  'vending machine',
  'vestment',
  'viaduct',
  'violin, fiddle',
  'volleyball',
  'waffle iron',
  'wall clock',
  'wallet, billfold, notecase, pocketbook',
  'wardrobe, closet, press',
  'warplane, military plane',
  'washbasin, handbasin, washbowl, lavabo, wash-hand basin',
  'washer, automatic washer, washing machine',
  'water bottle',
  'water jug',
  'water tower',
  'whiskey jug',
  'whistle',
  'wig',
  'window screen',
  'window shade',
  'Windsor tie',
  'wine bottle',
  'wing',
  'wok',
  'wooden spoon',
  'wool, woolen, woollen',
  'worm fence, snake fence, snake-rail fence, Virginia fence',
  'wreck',
  'yawl',
  'yurt',
  'web site, website, internet site, site',
  'comic book',
  'crossword puzzle, crossword',
  'street sign',
  'traffic light, traffic signal, stoplight',
  'book jacket, dust cover, dust jacket, dust wrapper',
  'menu',
  'plate',
  'guacamole',
  'consomme',
  'hot pot, hotpot',
  'trifle',
  'ice cream, icecream',
  'ice lolly, lolly, lollipop, popsicle',
  'French loaf',
  'bagel, beigel',
  'pretzel',
  'cheeseburger',
  'hotdog, hot dog, red hot',
  'mashed potato',
  'head cabbage',
  'broccoli',
  'cauliflower',
  'zucchini, courgette',
  'spaghetti squash',
  'acorn squash',
  'butternut squash',
  'cucumber, cuke',
  'artichoke, globe artichoke',
  'bell pepper',
  'cardoon',
  'mushroom',
  'Granny Smith',
  'strawberry',
  'orange',
  'lemon',
  'fig',
  'pineapple, ananas',
  'banana',
  'jackfruit, jak, jack',
  'custard apple',
  'pomegranate',
  'hay',
  'carbonara',
  'chocolate sauce, chocolate syrup',
  'dough',
  'meat loaf, meatloaf',
  'pizza, pizza pie',
  'potpie',
  'burrito',
  'red wine',
  'espresso',
  'cup',
  'eggnog',
  'alp',
  'bubble',
  'cliff, drop, drop-off',
  'coral reef',
  'geyser',
  'lakeside, lakeshore',
  'promontory, headland, head, foreland',
  'sandbar, sand bar',
  'seashore, coast, seacoast, sea-coast',
  'valley, vale',
  'volcano',
  'ballplayer, baseball player',
  'groom, bridegroom',
  'scuba diver',
  'rapeseed',
  'daisy',
  "yellow lady's slipper, yellow lady-slipper, Cypripedium calceolus, Cypripedium parviflorum",
  'corn',
  'acorn',
  'hip, rose hip, rosehip',
  'buckeye, horse chestnut, conker',
  'coral fungus',
  'agaric',
  'gyromitra',
  'stinkhorn, carrion fungus',
  'earthstar',
  'hen-of-the-woods, hen of the woods, Polyporus frondosus, Grifola frondosa',
  'bolete',
  'ear, spike, capitulum',
  'toilet tissue, toilet paper, bathroom tissue'};
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
void pcl::people::PersonCluster<PointT>::setObjectConfidence (float[] result)
{
  object_confidence_ = result;
}

template <typename PointT>
void pcl::people::PersonCluster<PointT>::setHeight (float height)
{
  height_ = height;
}

template <typename PointT>
void pcl::people::PersonCluster<PointT>::drawTBoundingBox (pcl::visualization::PCLVisualizer& viewer, int person_number, string classname)
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
void pcl::people::PersonCluster<PointT>::drawTBoundingBoxObject (pcl::visualization::PCLVisualizer& viewer, int person_number, int classindex)
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
  //      viewer.addText3D(, position, 0.1);
}


template <typename PointT>
pcl::people::PersonCluster<PointT>::~PersonCluster ()
{
  // Auto-generated destructor stub
}
#endif /* PCL_PEOPLE_PERSON_CLUSTER_HPP_ */
